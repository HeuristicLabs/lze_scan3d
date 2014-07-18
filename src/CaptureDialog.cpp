/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "CaptureDialog.hpp"

#include <QDesktopWidget>
#include <QMessageBox>
#include <QTime>

#include <iostream>

#include "Application.hpp"

CaptureDialog::CaptureDialog(QWidget * parent, Qt::WindowFlags flags): 
    QDialog(parent, flags),
    _projector(),
    _video_input(this),
    _capture(false),
    _session(),
    _wait_time(0),
    _total(0),
    _cancel(false)
{
    setupUi(this);
    camera_resolution_label->clear();

    connect(APP, SIGNAL(root_dir_changed(const QString&)), this, SLOT(_on_root_dir_changed(const QString&)));

    current_message_label->setVisible(false);
    progress_label->setVisible(false);
    progress_bar->setVisible(false);

    update_screen_combo();
    update_camera_combo();

    projector_patterns_spin->setValue(APP->config.value("capture/pattern_count", 10).toInt());
    camera_exposure_spin->setMaximum(2000);
    camera_exposure_spin->setValue(APP->config.value("capture/exposure_time", 500).toInt());
    output_dir_line->setText(APP->get_root_dir());

    //test buttons
    test_prev_button->setEnabled(false);
    test_next_button->setEnabled(false);

    //update projector view
    _projector.set_screen(screen_combo->currentIndex());

    //start video preview
    start_camera();

    // NAC
    _projector.choose_pattern_directory(parent);

}

CaptureDialog::~CaptureDialog()
{
    stop_camera();

    //save user selection
    QSettings & config = APP->config;
    int projector_screen = screen_combo->currentIndex();
    if (projector_screen>=0)
    {
        config.setValue("capture/projector_screen", projector_screen);
    }
    QString camera_name = camera_combo->currentText();
    if (!camera_name.isEmpty())
    {
        config.setValue("capture/camera_name", camera_name);
    }
    config.setValue("capture/pattern_count", projector_patterns_spin->value());
    config.setValue("capture/exposure_time", camera_exposure_spin->value());
}

void CaptureDialog::reset(void)
{
    progress_bar->setMaximum(0);
    current_message_label->clear();
    close_cancel_button->setText("Cancel");
    _cancel = false;
}

void CaptureDialog::finish(void)
{
    //progress_bar->setValue(_total);
    close_cancel_button->setText("Close");
}

void CaptureDialog::on_close_cancel_button_clicked(bool checked)
{
    if (close_cancel_button->text()=="Close")
    {
        accept();
    }
    else if (!_cancel)
    {
        _cancel = true;
    }
}

int CaptureDialog::update_screen_combo(void)
{
    //disable signals
    screen_combo->blockSignals(true);

    //save current value
    int current = screen_combo->currentIndex();

    //update combo
    QStringList list;
    screen_combo->clear();
    QDesktopWidget * desktop = QApplication::desktop();
    int screens =  desktop->screenCount();
    for (int i=0; i<screens; i++)
    {
        const QRect rect = desktop->screenGeometry(i);
        list.append(QString("Screen %1 [%2x%3]").arg(i).arg(rect.width()).arg(rect.height()));
    }
    screen_combo->addItems(list);

    //set current value
    int saved_value = APP->config.value("capture/projector_screen", 1).toInt();
    int default_screen = (saved_value<screen_combo->count() ? saved_value : 0);
    screen_combo->setCurrentIndex((-1<current && current<screen_combo->count() ? current : default_screen));

    //enable signals
    screen_combo->blockSignals(false);

    return screen_combo->count();
}

int CaptureDialog::update_camera_combo(void)
{
    //disable signals
    camera_combo->blockSignals(true);

    //save current value
    QString current = camera_combo->currentText();

    //update combo
    camera_combo->clear();
    camera_combo->addItems(_video_input.list_devices());

    //set current value
    int index = camera_combo->findText(current);
    if (index<0)
    {
        QString saved_value = APP->config.value("capture/camera_name").toString();
        index = camera_combo->findText(saved_value);
    }
    camera_combo->setCurrentIndex((index<0 ? 0 : index));

    //enable signals
    camera_combo->blockSignals(false);

    return camera_combo->count();
}

bool CaptureDialog::start_camera(void)
{
    int index = camera_combo->currentIndex();
    if (_video_input.get_camera_index()==index)
    {
        return true;
    }

    //busy cursor
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::processEvents();

    stop_camera();

    _video_input.set_camera_index(index);
    _video_input.start();
    _video_input.waitForStart();

    if (!_video_input.isRunning())
    {   //error
        //TODO: display error --------
        camera_resolution_label->clear();
    }

    //connect display signal now that we know it worked
    connect(&_video_input, SIGNAL(new_image(cv::Mat)), this, SLOT(_on_new_camera_image(cv::Mat)), Qt::DirectConnection);

    //restore regular cursor
    QApplication::restoreOverrideCursor();
    QApplication::processEvents();

    return true;
}

void CaptureDialog::stop_camera(void)
{
    //disconnect display signal first
    disconnect(&_video_input, SIGNAL(new_image(cv::Mat)), this, SLOT(_on_new_camera_image(cv::Mat)));

    //clean up
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor)); //busy cursor
    QApplication::processEvents(); //process pending signals
    camera_image->clear();
    camera_resolution_label->clear();
    
    //stop the thread
    if (_video_input.isRunning())
    {
        _video_input.stop();
        _video_input.wait();
    }

    //restore regular cursor
    QApplication::restoreOverrideCursor();
    QApplication::processEvents();
}

void CaptureDialog::_on_root_dir_changed(const QString & dirname)
{
    output_dir_line->setText(dirname);
}

void CaptureDialog::_on_new_camera_image(cv::Mat image)
{
    camera_image->setImage(image);
    camera_resolution_label->setText(QString("[%1x%2]").arg(image.cols).arg(image.rows));

    if (_capture)
    {   //save this image
        cv::imwrite(QString("%1/cam_%2.png").arg(_session).arg(_projector.get_current_pattern() + 1, 2, 10, QLatin1Char('0')).toStdString(), image);
        _capture = false;
        _projector.clear_updated();
    }
}

void CaptureDialog::_on_new_projector_image(QPixmap image)
{
    projector_image->setPixmap(image);
}

void CaptureDialog::on_screen_combo_currentIndexChanged(int index)
{
    _projector.set_screen(index);
}

void CaptureDialog::on_camera_combo_currentIndexChanged(int index)
{
    camera_resolution_label->clear();
    start_camera();
}

void CaptureDialog::on_output_dir_line_textEdited(const QString & text)
{
    APP->set_root_dir(text);
}

void CaptureDialog::on_output_dir_button_clicked(bool checked)
{
    APP->change_root_dir(this);
}

void CaptureDialog::wait(int msecs)
{
    QTime timer;
    timer.start();
    while (timer.elapsed()<msecs)
    {   
        QApplication::processEvents();
    }
}

void CaptureDialog::on_capture_button_clicked(bool checked)
{
    //check camera
    if (!_video_input.isRunning())
    {
        QMessageBox::critical(this, "Error", "Camera is not ready");
        return;
    }

    //make output dir
    _session = APP->get_root_dir()+"/"+QDateTime::currentDateTime().toString("yyyy-MMM-dd_hh.mm.ss.zzz");
    QDir session_dir;
    if (!session_dir.mkpath(_session))
    {
        QMessageBox::critical(this, "Error", "Cannot create output directory:\n"+_session);
        std::cout << "Failed to create directory: " << _session.toStdString() << std::endl;
        return;
    }

    //disable GUI interaction
    projector_group->setEnabled(false);
    camera_group->setEnabled(false);
    other_group->setEnabled(false);
    capture_button->setEnabled(false);
    close_cancel_button->setEnabled(false);

    _capture = false;
    _wait_time = camera_exposure_spin->value();

    //connect projector display signal
    connect(&_projector, SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));

    //open projector
    _projector.set_pattern_count(projector_patterns_spin->value());
    _projector.start();

    //save projector resolution and settings
    _projector.save_info(QString("%1/projector_info.txt").arg(_session));

    //init time
    wait(_wait_time);

    while (!_projector.finished())
    {

	// NAC
	await_udp_sync_msg();

        _projector.next();
        //QMessageBox::critical(this, "Next", "Continue");

        //wait for projector
        while (!_projector.is_updated())
        {   
            QApplication::processEvents();
        }

        //pause so the screen gets updated
        wait(_wait_time);

        //capture
        _capture = true;

        //wait for camera
        while (_capture)
        {   
            QApplication::processEvents();
        }

    }

    //close projector
    _projector.stop();

    //disconnect projector display signal
    disconnect(&_projector, SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));
   

    //re-read images
    APP->set_root_dir(APP->get_root_dir());

    //enable GUI interaction
    projector_group->setEnabled(true);
    camera_group->setEnabled(true);
    other_group->setEnabled(true);
    capture_button->setEnabled(true);
    close_cancel_button->setEnabled(true);

    //TODO: override window close button or allow to close/cancel while capturing
}

// NAC
void CaptureDialog::send_udp_sync_msg() const {
  // hostname
  // port
}

// NAC
// see http://www.microhowto.info/howto/listen_for_and_receive_udp_datagrams_in_c.html
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
void CaptureDialog::await_udp_sync_msg() const {

// from http://stackoverflow.com/questions/12515005/receiving-udp-broadcast

sockaddr_in si_me, si_other;
int s;
assert((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))!=-1);
int broadcast=1;

setsockopt(s, SOL_SOCKET, SO_BROADCAST,
            &broadcast, sizeof broadcast);

memset(&si_me, 0, sizeof(si_me));
si_me.sin_family = AF_INET;
si_me.sin_port = htons(port_);
si_me.sin_addr.s_addr = INADDR_ANY;

assert(::bind(s, (sockaddr *)&si_me, sizeof(sockaddr))!=-1);

//while(1)
//{
    char buf[10000];
    unsigned slen=sizeof(sockaddr);
    recvfrom(s, buf, sizeof(buf)-1, 0, (sockaddr *)&si_other, &slen);

    printf("recv: %s\n", buf);
    //}

/*
const char* hostname=0; // wildcard
const char* portname="daytime";
struct addrinfo hints;
memset(&hints,0,sizeof(hints));
hints.ai_family=AF_UNSPEC;
hints.ai_socktype=SOCK_DGRAM;
hints.ai_protocol=0;
hints.ai_flags=AI_PASSIVE|AI_ADDRCONFIG;
struct addrinfo* res=0;
int err=getaddrinfo(hostname,portname,&hints,&res);
if (err!=0) {
    die("failed to resolve local socket address (err=%d)",err);
}

int fd=socket(res->ai_family,res->ai_socktype,res->ai_protocol);
if (fd==-1) {
    die("%s",strerror(errno));
}

if (bind(fd,res->ai_addr,res->ai_addrlen)==-1) {
    die("%s",strerror(errno));
}

freeaddrinfo(res);

char buffer[549];
struct sockaddr_storage src_addr;
socklen_t src_addr_len=sizeof(src_addr);
ssize_t count=recvfrom(fd,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
if (count==-1) {
    die("%s",strerror(errno));
} else if (count==sizeof(buffer)) {
    warn("datagram too large for buffer: truncated");
} else {
//handle_datagram(buffer,count);
    warn("got datagram; but haven't written code to process it yet!");
}
*/
}

void CaptureDialog::on_test_check_stateChanged(int state)
{
    //adjust the GUI
    bool checked = (state==Qt::Checked);
    test_prev_button->setEnabled(checked);
    test_next_button->setEnabled(checked);
    capture_button->setEnabled(!checked);
    screen_combo->setEnabled(!checked);
    projector_patterns_spin->setEnabled(!checked);

    if (checked)
    {   //start preview

        //connect projector display signal
        connect(&_projector, SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));

        //open projector
        _projector.set_pattern_count(projector_patterns_spin->value());
        _projector.start();
        _projector.next();
    }
    else
    {   //stop preview

        //close projector
        _projector.stop();

        //disconnect projector display signal
        disconnect(&_projector, SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));
    }
}

void CaptureDialog::on_test_prev_button_clicked(bool checked)
{
    _projector.clear_updated();
    _projector.prev();
}

void CaptureDialog::on_test_next_button_clicked(bool checked)
{
    _projector.clear_updated();
    _projector.next();
}
