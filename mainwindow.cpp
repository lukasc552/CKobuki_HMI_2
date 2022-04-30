#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "QPainter"
#include "math.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>


///konstruktor aplikacie, nic co tu je nevymazavajte, ak potrebujete, mozete si tu pridat nejake inicializacne parametre
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    robotX=0;
    robotY=0;
    robotFi=0;

    showCamera=true;
    showLidar=true;
    showSkeleton=false;
    applyDelay=false;
    dl=0;
    stopall=1;
    prvyStart=true;
    updateCameraPicture=0;
    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::move(std::thread(f2));


    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::move(std::thread(f3));

    //--ak by ste nahodou chceli konzolu do ktorej mozete vypisovat cez std::cout, odkomentujte nasledujuce dva riadky
   // AllocConsole();
   // freopen("CONOUT$", "w", stdout);

//    mapHeight = 6; // metrov
//    mapWidth = 6;  // metrov
//    step = 0.1;
    mapHeight = 12; // metrov
    mapWidth = 12;  // metrov
    step = 0.05;
    gridRows = mapHeight/step;
    gridCols = mapWidth/step;

    for (int i = 0; i<gridRows; i++) {
        for (int j = 0; j<gridCols; j++) {
            grid[i][j] = 0;
        }
    }

    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });



        Imager.start();
//    applyDelay = true;
}

void MainWindow::initialValues(){
    controlData = {0};
    controlData.oldEncoderLeft = sens.EncoderLeft;
    controlData.oldEncoderRight = sens.EncoderRight;
    controlData.initGyroAngle = sens.GyroAngle;

    controlData.rampLimit = 10;
    controlData.Kp_dist = 600;
    controlData.rampLimitAngle = 0.4;
    controlData.Kp_angle = 1.6;
    controlData.pa1 = 4*PI/180;
    controlData.pa2 = 7*PI/180;
    controlData.deathAngle = controlData.pa2;

    done_goals.push_back({0.0, 0.0});

//    if(fusionData.prvyStart){
//        fusionData.altLidar = 0.21;
//        fusionData.altCamera = 0.15;
//        fusionData.Y = fusionData.altLidar - fusionData.altCamera;
//        fusionData.camWidthAngle = (54-2)*PI/180;
//        fusionData.camHeightAngle = 40*PI/180;
//        fusionData.f = 628.036;

//        fusionData.prvyStart = false;
//    }
}


///kamera nema svoju vlastnu funkciu ktora sa vola, ak chcete niekde pouzit obrazok, aktualny je v premennej
/// robotPicture alebo ekvivalent AutonomousrobotPicture
/// pozor na synchronizaciu, odporucam akonahle chcete robit nieco s obrazkom urobit si jeho lokalnu kopiu
/// cv::Mat frameBuf; robotPicture.copyTo(frameBuf);


//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::darkGray);
    QPen pero_walls;
    pero_walls.setStyle(Qt::SolidLine);
    pero_walls.setWidth(1);
    pero_walls.setColor(Qt::darkYellow);

    QPen pero_lidar;
    pero_lidar.setStyle(Qt::SolidLine);
    pero_lidar.setWidth(1);
    pero_lidar.setColor(Qt::blue);

    QPen pero_robot;
    pero_robot.setStyle(Qt::SolidLine);
    pero_robot.setWidth(2);
    pero_robot.setColor(Qt::cyan);

    QPen pero_goals;
    pero_goals.setStyle(Qt::SolidLine);
    pero_goals.setWidth(3);
    pero_goals.setColor(Qt::red);

    QPen pero_lines;
    pero_lines.setStyle(Qt::SolidLine);
    pero_lines.setWidth(2);
    pero_lines.setColor(Qt::yellow);

    QPen pero_msg;
    pero_msg.setStyle(Qt::SolidLine);
    pero_msg.setWidth(1);
    pero_msg.setColor(Qt::blue);

    rect = ui->map_frame->geometry();
    painter.drawRect(rect);
    painter.setBrush(Qt::black);
    QRect camera_rect = ui->camera_frame->geometry();
    QRect message_rect = ui->message_frame->geometry();
    QRect stop_msg_rect = ui->stop_frame->geometry();

    if(msg){
        painter.setBrush(Qt::white);
        painter.drawRect(message_rect);
        painter.setPen(pero_msg);
        painter.setFont(QFont("Times", 10, QFont::Bold));
//        painter.drawText(QPoint(message_rect.topLeft().x() + 40, message_rect.topLeft().y()+30), message);
        painter.drawText(message_rect, Qt::AlignHCenter | Qt::AlignVCenter, message);
        check++;
        if(check>20){
//            cout << "update..." << endl;

            check = 0;
            msg = false;
        }
        painter.setBrush(Qt::black);
    }

    if(stop){
        painter.setBrush(Qt::white);
        painter.drawRect(stop_msg_rect);
        QPen stop_pen;
        stop_pen.setWidth(2);
        stop_pen.setColor(Qt::red);
        painter.setPen(stop_pen);
        painter.setFont(QFont("Times", 15, QFont::Bold));
//        painter.drawText(QPoint(stop_msg_rect.topLeft().x() + 15, stop_msg_rect.topLeft().y()+30), "Emergency STOP");
        painter.drawText(stop_msg_rect, Qt::AlignHCenter | Qt::AlignVCenter, "Emergency STOP");
        painter.setBrush(Qt::black);
    }

    if(updateCameraPicture==1 && showCamera==true)
    {
        updateCameraPicture=0;
//        int int_camera_size = 3*camera_rect.width() < 4*camera_rect.height() ? camera_rect.width() : camera_rect.height();
        QRect croped_camera;
        int cam_rect_width;
        int cam_rect_height;
        if(3*camera_rect.width() < 4*camera_rect.height()){
            cam_rect_width = camera_rect.width();
            cam_rect_height = (camera_rect.width()/4)*3;
        }else{
            cam_rect_height = camera_rect.height();
            cam_rect_width = (camera_rect.height()/3)*4;
        }
        int cam_shift_x = (camera_rect.width() - cam_rect_width)/2;
        int cam_shift_y = (camera_rect.height() - cam_rect_height)/2;
        croped_camera.setRect(camera_rect.topLeft().x()+cam_shift_x, camera_rect.topLeft().y()+cam_shift_y, cam_rect_width, cam_rect_height);

//        cv::resize(robotPicture, robotPicture, cv::Size(camera_rect.width(), camera_rect.height()), 0, 0, cv::INTER_CUBIC);
        cv::resize(robotPicture, robotPicture, cv::Size(croped_camera.width(), croped_camera.height()), 0, 0, cv::INTER_CUBIC);

        QImage imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
//        painter.drawImage(20, 120, imgIn);
//        painter.drawImage(camera_rect.topLeft().x(), camera_rect.topLeft().y()+5, imgIn);
        painter.drawImage(croped_camera.topLeft().x(), croped_camera.topLeft().y()+5, imgIn);
      //  cv::imshow("client",robotPicture);
    }
    applyDelay = true;
    if(updateLaserPicture==1 && showLidar==true)
    {
        updateLaserPicture = 0;

        int interest_size = rect.width() < rect.height() ? rect.width() : rect.height();
//        temp_rect = rect;
        shift_x = (rect.width() - interest_size)/2;
        shift_y = (rect.height() - interest_size)/2;

        crop.setRect(rect.topLeft().x()+shift_x, rect.topLeft().y()+shift_y, interest_size, interest_size);
        painter.drawRect(crop);
        rect = crop;
        /// ****************
        ///you can change pen or pen color here if you want
        /// ****************
        ///

        painter.setPen(pero_walls);

        for(int ii = 0; ii< gridRows; ii++){
            for(int jj = 0; jj<gridCols; jj++){

                if(grid[ii][jj] == 1){

                    X_Y xyg = getXYFromMapIdx({ii, jj});

                    int x = coordsToPixels(xyg.x, xyg.y).j;
                    int y = coordsToPixels(xyg.x, xyg.y).i;

                    if(crop.contains(x, y)){
                        painter.drawEllipse(QPoint(x, y), 2, 2);
                    }
                }
            }
        }

        painter.setPen(pero_lidar);

        for(int k = 0; k<paintLaserData.numberOfScans; k++){
            double distance = paintLaserData.Data[k].scanDistance/1000;
            double alpha = delayedControlData.phi + (360 - paintLaserData.Data[k].scanAngle)*PI/180;

            double xg = delayedControlData.x + distance * cos(alpha);
            double yg = delayedControlData.y + distance * sin(alpha);

            int x = coordsToPixels(xg, yg).j;
            int y = coordsToPixels(xg, yg).i;

            if(crop.contains(x, y)){
                painter.drawEllipse(QPoint(x, y), 2, 2);
            }

        }

//        QVector<QPoint> qvector;
        painter.setPen(pero_goals);
        for(X_Y xy : fifo_array){
            int x = coordsToPixels(xy.x, xy.y).j;
            int y = coordsToPixels(xy.x, xy.y).i;

            if(crop.contains(x, y)){
                painter.drawEllipse(QPoint(x, y), 2, 2);
            }
        }
        pero_goals.setColor(Qt::white);
        painter.setPen(pero_goals);
//        for(X_Y xy : done_goals){
        for(int k = 1;k< done_goals.size(); k++){
            X_Y xy_old = done_goals.at(k-1);
            X_Y xy = done_goals.at(k);
            int x_old = coordsToPixels(xy_old.x, xy_old.y).j;
            int y_old = coordsToPixels(xy_old.x, xy_old.y).i;
            int x = coordsToPixels(xy.x, xy.y).j;
            int y = coordsToPixels(xy.x, xy.y).i;

            painter.drawLine(QPoint(x, y), QPoint(x_old, y_old));
//            if(crop.contains(x, y)){
//                painter.drawEllipse(QPoint(x, y), 2, 2);
//            }
        }

//        painter.setPen(pero_lines);
//        painter.drawLines(qvector);

        painter.setPen(pero_robot);
        int px_polomer_robota = interest_size/45;
        int px_xr = coordsToPixels(delayedControlData.x, delayedControlData.y).j;
        int px_yr = coordsToPixels(delayedControlData.x, delayedControlData.y).i;
//        int px_xr = rect.topLeft().x() + (rect.width()/5 + ((controlData.x/(mapWidth/1.6)) * rect.width()));
//        int px_yr = rect.bottomLeft().y() - (rect.height()/4 + ((controlData.y/(mapHeight/1.8)) * rect.height()));

        painter.drawEllipse(QPoint(px_xr, px_yr), px_polomer_robota, px_polomer_robota);
        painter.drawLine(QPoint(px_xr, px_yr), QPoint(px_xr + (px_polomer_robota * cos(delayedControlData.phi)), px_yr - (px_polomer_robota * sin(delayedControlData.phi))));

    }
    if(updateSkeletonPicture==1 && showSkeleton==true)
    {
        // nic tu nieje
    }
}


//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    ///
    ///
    ///
    ///
    ///
    ///



    if(prvyStart)
    {

        GyroUholOld=sens.GyroAngle;
        PomEncoderL= sens.EncoderLeft;
        PomEncoderR= sens.EncoderRight;


        deltaUhol=0.0;
        prvyStart=false;
    }

    calcDelayedDifferences(sens);
    calcDelayedLocalisation(sens);

    PomEncoderL=sens.EncoderLeft;
    if(dl%10==0)
    {
        ///toto je skaredy kod. rozumne je to posielat do ui cez signal slot..
//        ui->lineEdit->setText(QString::number(robotX));
//        ui->lineEdit_2->setText(QString::number(robotY));
//        ui->lineEdit_3->setText(QString::number(robotFi));
    }
    dl++;
}

void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));

//    if(mapSampleStep%10==0/* && readLaserToMap*/){
//        compMap(paintLaserData);
//    }
//    mapSampleStep++;

    updateLaserPicture=1;
//    update();
}

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{

    paintThisLidar(laserData);

    if(mapSampleStep%10==0/* && readLaserToMap*/){
        compMap(laserData);
    }
    mapSampleStep++;
    //priklad ako zastavit robot ak je nieco prilis blizko
//    if(5<laserData.Data[0].scanDistance && laserData.Data[0].scanDistance<100)
//    {
//        if(doControl){
//            fifo_array.erase(fifo_array.begin());
////            if(fifo_array.size())
//        }

//        sendRobotCommand(ROBOT_STOP);
//    }

    for(int i = 0; i<laserData.numberOfScans; i++){
        if(5<laserData.Data[i].scanDistance && laserData.Data[i].scanDistance<criticalDistLidarToWall) // 500
        {
            dangerZoneDelayed = true;
//            if(doControl){
//                fifo_array.erase(fifo_array.begin());
//                cout<<"Nedosiahnutelny bod" << endl;
//            }
//            sendRobotCommand(ROBOT_STOP);
            break;
        }
        if(dangerZoneDelayed){
            dangerZoneDelayed = false;
            removedDelayed--;
        }
    }

    if(dangerZoneDelayed && removedDelayed==0){
//        fifo_array.erase(fifo_array.begin());
        removedDelayed++;
//        cout<<"Nedosiahnutelny bod" << endl;
        msg = true;
        message = ">> Nedosiahnutelny bod! <<";

//        appendLogToPlainText(QString("Nedosiel som"));
    }
    ///PASTE YOUR CODE HERE
    /// ****************
    /// mozne hodnoty v return
    /// ROBOT_VPRED
    /// ROBOT_VZAD
    /// ROBOT_VLAVO
    /// ROBOT_VPRAVO
    /// ROBOT_STOP
    /// ROBOT_ARC
    ///
    /// ****************

    return -1;
}

//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    ///

    for(int i = 0; i<laserData.numberOfScans; i++){
        if(5<laserData.Data[i].scanDistance && laserData.Data[i].scanDistance<criticalDistLidarToWall) // 500
        {
            dangerZone = true;
//            if(doControl){
//                fifo_array.erase(fifo_array.begin());
//                cout<<"Nedosiahnutelny bod" << endl;
//            }
            sendRobotCommand(ROBOT_STOP);
            break;
        }
        if(dangerZone){
            dangerZone = false;
            removed--;
        }
    }

    if(doControl && dangerZone && removed == 0){
        fifo_array.erase(fifo_array.begin());
        removed++;
        cout<<"Nedosiahnutelny bod" << endl;
//        msg = true;
//        message = ">> Nedosiahnutelny bod! <<";

//        appendLogToPlainText(QString("Nedosiel som"));
    }


//    if(mapSampleStep%10==0/* && readLaserToMap*/){
//        compMap(laserData);
//    }
//    mapSampleStep++;

    return -1;
}

//void MainWindow::appendLogToPlainText(QString string){
//    QPlainTextEdit *qplain = ui->plainTextEdit;
//    qplain->setReadOnly(true);
////    qplain->focusProxy()

//    qplain->appendPlainText(string);
//    QTextCursor c = qplain->textCursor();
//    c.movePosition(QTextCursor::End);
//    qplain->setTextCursor(c);
//}

//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    ///

    if (stop) return;

    calcDifferences(sens);
    calcLocalisation(sens);

    if(dangerZone){
        sendRobotCommand(ROBOT_VZAD, -200);
    }else{

        if(!fifo_array.empty()){
            controlData.x_set = fifo_array.front().x;
            controlData.y_set = fifo_array.front().y;

            double angle_to_target = atan2((controlData.y_set - controlData.y), (controlData.x_set - controlData.x));
            double error_angle = angle_to_target - controlData.phi;
            if(error_angle < -PI) error_angle += 2*PI;
            if(error_angle > PI) error_angle -= 2*PI;
    //        std::cout << "X: " << controlData.x << ", Y: " << controlData.y << ", phi: " << controlData.phi << std::endl;
    //            std::cout << "Angle to target: " << angle_to_target << std::endl;
    //            std::cout << "Error angle: " << error_angle << std::endl;
            if(abs(error_angle) > controlData.deathAngle)
            {
                controlData.deathAngle = controlData.pa1;
    //            std::cout << ">>>>>>>>>>>>>> Otacam sa <<<<<<<<<<<<<<" << std::endl;

                controlData.output = 0;
                regulateAngle(error_angle);

                std::vector<unsigned char> mess=robot.setRotationSpeed(controlData.outputAngle);
                sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen);
            }
            if(abs(error_angle) <= controlData.deathAngle){
    //            std::cout << ">> Idem priamo <<" << std::endl;

                controlData.deathAngle = controlData.pa2;
                double measuredValue = sqrt(pow(controlData.x_set-controlData.x,2) + pow(controlData.y_set-controlData.y,2));
                double error_dist = abs(controlData.setpoint - measuredValue);

                controlData.outputAngle = 0;
                regulatePosition(error_dist);
    //            std::cout << "Output: " << controlData.output << std::endl;
    //            std::cout << "error_distance: " << error_dist << std::endl;
                std::vector<unsigned char> mess;
                if(error_dist < 0.015){
                    mess=robot.setTranslationSpeed(0);
                    done_goals.push_back(fifo_array.front());
                    fifo_array.erase(fifo_array.begin());
                }else{
                    mess=robot.setTranslationSpeed(controlData.output);
                }
                sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen);

            }
            ending = true;
        }else{
            doControl = false;
            if(ending){
    //            writeMapToFile();
                ending = false;
                sendRobotCommand(ROBOT_STOP);
                std::cout << ">> Nerobim -> Koniec <<" << std::endl;
                if(!dangerZone){
                    msg = true;
                    message = ">> Uloha splnena <<";
                }
            }
        }
    }
}

void MainWindow::calcDifferences(TKobukiData &sens){
    // Pretecenie Encoderov
    controlData.diffEncRight = (sens.EncoderRight + SHORT_HALF_SIZE - controlData.oldEncoderRight) % SHORT_SIZE - SHORT_HALF_SIZE;
    controlData.diffEncLeft = (sens.EncoderLeft + SHORT_HALF_SIZE - controlData.oldEncoderLeft) % SHORT_SIZE - SHORT_HALF_SIZE;

    controlData.oldEncoderLeft = sens.EncoderLeft;
    controlData.oldEncoderRight = sens.EncoderRight;
}

void MainWindow::calcLocalisation(TKobukiData &sens){
    double l_rk = robot.getTickToMeter() * controlData.diffEncRight;
    double l_lk = robot.getTickToMeter() * controlData.diffEncLeft;
    controlData.phi = ((sens.GyroAngle - controlData.initGyroAngle)/100)*PI/180;

    double lk;
    lk = (l_rk+l_lk)/2;
    controlData.x = controlData.x_old + lk * cos(controlData.phi_old); //  sin(phi_new)
    controlData.y = controlData.y_old + lk * sin(controlData.phi_old);

    controlData.phi_old = controlData.phi;
    controlData.x_old = controlData.x;
    controlData.y_old = controlData.y;
}

void MainWindow::calcDelayedDifferences(TKobukiData &sens){
    // Pretecenie Encoderov
    delayedControlData.diffEncRight = (sens.EncoderRight + SHORT_HALF_SIZE - delayedControlData.oldEncoderRight) % SHORT_SIZE - SHORT_HALF_SIZE;
    delayedControlData.diffEncLeft = (sens.EncoderLeft + SHORT_HALF_SIZE - delayedControlData.oldEncoderLeft) % SHORT_SIZE - SHORT_HALF_SIZE;

    delayedControlData.oldEncoderLeft = sens.EncoderLeft;
    delayedControlData.oldEncoderRight = sens.EncoderRight;
}

void MainWindow::calcDelayedLocalisation(TKobukiData &sens){
    double l_rk = robot.getTickToMeter() * delayedControlData.diffEncRight;
    double l_lk = robot.getTickToMeter() * delayedControlData.diffEncLeft;
    delayedControlData.phi = ((sens.GyroAngle - delayedControlData.initGyroAngle)/100)*PI/180;

    double lk;
    lk = (l_rk+l_lk)/2;
    delayedControlData.x = delayedControlData.x_old + lk * cos(delayedControlData.phi_old); //  sin(phi_new)
    delayedControlData.y = delayedControlData.y_old + lk * sin(delayedControlData.phi_old);

    delayedControlData.phi_old = delayedControlData.phi;
    delayedControlData.x_old = delayedControlData.x;
    delayedControlData.y_old = delayedControlData.y;
}

void MainWindow::regulateAngle(double error_angle){
    if(controlData.Kp_angle * error_angle >= controlData.rampLimitAngle + controlData.outputAngle){
        controlData.outputAngle += controlData.rampLimitAngle;
    }else if(controlData.Kp_angle * error_angle <= controlData.outputAngle - controlData.rampLimitAngle){
        controlData.outputAngle -= controlData.rampLimitAngle;
    }else{
        controlData.outputAngle = (controlData.Kp_angle * error_angle);
    }

    if (controlData.outputAngle > MAX_ANGLE_SPEED) controlData.outputAngle = MAX_ANGLE_SPEED;
    if (controlData.outputAngle < -MAX_ANGLE_SPEED) controlData.outputAngle = -MAX_ANGLE_SPEED;
//    std::cout << "out_angle: " << out_angle << std::endl;
}


void MainWindow::regulatePosition(double error_distance){
    if(controlData.Kp_dist * error_distance > controlData.rampLimit + controlData.output){
        controlData.output += controlData.rampLimit;
    }else{
        controlData.output = controlData.Kp_dist * error_distance;
    }
    if(controlData.output > MAX_SPEED) controlData.output = MAX_SPEED;
}

void MainWindow::compMap(LaserMeasurement &laserData){

    if(delayedControlData.diffEncLeft == delayedControlData.diffEncRight /*robot.leftForward == true && robot.rightForward == true*/){
        for (int k =0; k<laserData.numberOfScans; k++) {
            double d = laserData.Data[k].scanDistance/1000;
            if(d < 0.1 || d > 2.2) continue;
            double alpha = delayedControlData.phi + (360 - laserData.Data[k].scanAngle)*PI/180;

            double xg = delayedControlData.x + d * cos(alpha);
            double yg = delayedControlData.y + d * sin(alpha);

            IJ_idx place = placeInGrid({xg, yg});
            grid[place.i][place.j] = 1;
        }

    }
}

IJ_idx MainWindow::placeInGrid(X_Y xyg){
    IJ_idx result;
    result.j = ((gridCols/2)-1) + int(xyg.x/step);
    result.i = ((gridRows/2)-1) - int(xyg.y/step);

//    result.j = int(xyg.x/step);
//    result.i = (gridRows-1) - int(xyg.y/step);
    return result;
}

X_Y MainWindow::getXYFromMapIdx(IJ_idx idxs){
    X_Y result;
    int xx = step*((idxs.j + 1) - (gridCols/2))*100;
    int yy = (-1)*step*((idxs.i + 1) - (gridRows/2))*100;
//    int xx = step * idxs.j * 100;
//    int yy = (gridRows - idxs.i - 1) * step * 100;
    result.x = xx/100.0;
    result.y = yy/100.0; // +s/2
    return result;
}


void MainWindow::mousePressEvent(QMouseEvent* event){
//    if(ui->map_frame->geometry().contains(event->pos())){
    if(crop.contains(event->pos())){
        qpoint = ui->map_frame->mapFromGlobal(QCursor::pos());
        cout<< "Klikol som do map-frame-u" << endl;
        cout << "x: " << qpoint.x() << "; y: " <<qpoint.y() << endl;
        X_Y xy = pixelToCoords(qpoint.x()-shift_x, qpoint.y()-shift_y-12);
        fifo_array.push_back(xy);
        doControl = true;
    }
}


X_Y MainWindow::pixelToCoords(int px, int py){

    X_Y result;
    result.x = ((px - crop.width()/5)*mapWidth)/(1.6*crop.width());
    result.y = mapHeight/4 - ((py - crop.height()/4)*mapHeight)/(1.8*crop.height());
//    result.y = ((py - rect.bottomLeft().y() + (rect.height()/4))*mapHeight)/(1.8*rect.height());
    return result;
}

IJ_idx MainWindow::coordsToPixels(double x, double y){
    IJ_idx result;
    result.j = crop.topLeft().x() + (crop.width()/5 + ((x/(mapWidth/1.6)) * crop.width()));
    result.i = crop.bottomLeft().y() - (crop.height()/4 + ((y/(mapHeight/1.8)) * crop.height()));
    return result;
}

//X_Y MainWindow::pixelToCoords(int px, int py, int interest_size){

//    X_Y result;
//    result.x = ((px - rect.width()/5)*mapWidth)/(1.8*rect.width());
//    result.y = mapHeight/4 - ((py - rect.height()/4)*mapHeight)/(1.8*rect.height());
////    result.y = ((py - rect.bottomLeft().y() + (rect.height()/4))*mapHeight)/(1.8*rect.height());
//    return result;
//}

//IJ_idx MainWindow::coordsToPixels(double x, double y, int interest_size){
//    IJ_idx result;
//    result.j = crop.topLeft().x() + (crop.width()/5 + ((x/(mapWidth/1.8)) * crop.width()));
//    result.i = crop.bottomLeft().y() - (crop.height()/4 + ((y/(mapHeight/1.8)) * crop.height()));
//    return result;
//}




///funkcia co sa zavola ked stlacite klavesu na klavesnici..
/// pozor, ak niektory widget akceptuje klavesu, sem sa nemusite (ale mozete) dostat
/// zalezi na to ako konkretny widget spracuje svoj event
void MainWindow::keyPressEvent(QKeyEvent* event)
{
    //pre pismena je key ekvivalent ich ascii hodnoty
    //pre ine klavesy pozrite tu: https://doc.qt.io/qt-5/qt.html#Key-enum
    std::cout<<event->key()<<std::endl;


}
//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody































































MainWindow::~MainWindow()
{
    sendRobotCommand(ROBOT_STOP);
    stopall=0;
    laserthreadHandle.join();
    robotthreadHandle.join();
    skeletonthreadHandle.join();
    delete ui;
}









void MainWindow::robotprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    bool in = true;
    while(stopall==1)
    {

        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval=robot.fillData(sens,(unsigned char*)buff);

        if(in){
            initialValues();
            in = false;
        }

        if(returnval==0)
        {
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();

            autonomousrobot(sens);

            if(applyDelay==true)
            {
                struct timespec t;
                RobotData newcommand;
                newcommand.sens=sens;
                //    memcpy(&newcommand.sens,&sens,sizeof(TKobukiData));
                //        clock_gettime(CLOCK_REALTIME,&t);
                newcommand.timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                sensorQuerry.push_back(newcommand);
                for(int i=0;i<sensorQuerry.size();i++)
                {
                    if(( std::chrono::duration_cast<std::chrono::nanoseconds>(timestampf-sensorQuerry[i].timestamp)).count()>(2.5*1000000000))
                    {
                        localrobot(sensorQuerry[i].sens);
                        sensorQuerry.erase(sensorQuerry.begin()+i);
                        i--;
                        break;

                    }
                }

            }
            else
            {
                sensorQuerry.clear();
                localrobot(sens);
            }
        }


    }

    std::cout<<"koniec thread2"<<std::endl;
}
/// vravel som ze vas to nemusi zaujimat. tu nic nieje
/// nosy litlle bastard
void MainWindow::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(stopall==1)
    {

        if ((las_recv_len = recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        int returnValue=autonomouslaser(measure);

        if(applyDelay==true)
        {
            struct timespec t;
            LidarVector newcommand;
            memcpy(&newcommand.data,&measure,sizeof(LaserMeasurement));
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            lidarQuerry.push_back(newcommand);
            for(int i=0;i<lidarQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-lidarQuerry[i].timestamp)).count()>(2.5*1000000000))
                {
                    returnValue=locallaser(lidarQuerry[i].data);
                    if(returnValue!=-1)
                    {
                        //sendRobotCommand(returnValue);
                    }
                    lidarQuerry.erase(lidarQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


            returnValue=locallaser(measure);
            if(returnValue!=-1)
            {
                //sendRobotCommand(returnValue);
            }
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}



void MainWindow::sendRobotCommand(char command,double speed,int radius)
{
    globalcommand=command;
//    if(applyDelay==false)
//    {

        std::vector<unsigned char> mess;
        switch(command)
        {
        case  ROBOT_VPRED:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VZAD:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VLAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_VPRAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_STOP:
            mess=robot.setTranslationSpeed(0);
            break;
        case ROBOT_ARC:
            mess=robot.setArcSpeed(speed,radius);
            break;


        }
        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

        }
//    }
//    else
//    {
//        struct timespec t;
//        RobotCommand newcommand;
//        newcommand.command=command;
//        newcommand.radius=radius;
//        newcommand.speed=speed;
//        //clock_gettime(CLOCK_REALTIME,&t);
//        newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//        commandQuery.push_back(newcommand);
//    }
}



//void MainWindow::autonomousRobotCommand(char command,double speed,int radius)
//{
//    return;
//    std::vector<unsigned char> mess;
//    switch(command)
//    {
//    case  ROBOT_VPRED:
//        mess=robot.setTranslationSpeed(speed);
//        break;
//    case ROBOT_VZAD:
//        mess=robot.setTranslationSpeed(speed);
//        break;
//    case ROBOT_VLAVO:
//        mess=robot.setRotationSpeed(speed);
//        break;
//    case ROBOT_VPRAVO:
//        mess=robot.setRotationSpeed(speed);
//        break;
//    case ROBOT_STOP:
//        mess=robot.setTranslationSpeed(0);
//        break;
//    case ROBOT_ARC:
//        mess=robot.setArcSpeed(speed,radius);
//        break;

//    }
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {

//    }
//}
//void MainWindow::robotexec()
//{


//    if(applyDelay==true)
//    {
//        struct timespec t;

//        // clock_gettime(CLOCK_REALTIME,&t);
//        auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//        for(int i=0;i<commandQuery.size();i++)
//        {
//       //     std::cout<<(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()<<std::endl;
//            if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()>(2.5*1000000000))
//            {
//                char cmd=commandQuery[i].command;
//                std::vector<unsigned char> mess;
//                switch(cmd)
//                {
//                case  ROBOT_VPRED:
//                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
//                    break;
//                case ROBOT_VZAD:
//                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
//                    break;
//                case ROBOT_VLAVO:
//                    mess=robot.setRotationSpeed(commandQuery[i].speed);
//                    break;
//                case ROBOT_VPRAVO:
//                    mess=robot.setRotationSpeed(commandQuery[i].speed);
//                    break;
//                case ROBOT_STOP:
//                    mess=robot.setTranslationSpeed(0);
//                    break;
//                case ROBOT_ARC:
//                    mess=robot.setArcSpeed(commandQuery[i].speed,commandQuery[i].radius);
//                    break;

//                }
//                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//                {

//                }
//                commandQuery.erase(commandQuery.begin()+i);
//                i--;

//            }
//        }
//    }
//}



//void MainWindow::on_pushButton_8_clicked()//forward
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=100;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_10_clicked()//right
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=-20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    // clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_11_clicked()//back
//{
//    CommandVector help;
//    help.command.commandType=1;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=0;
//    help.command.desiredDist=-100;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_9_clicked()//left
//{
//    CommandVector help;
//    help.command.commandType=2;
//    help.command.actualAngle=0;
//    help.command.actualDist=0;
//    help.command.desiredAngle=20;
//    help.command.desiredDist=0;
//    struct timespec t;

//    //   clock_gettime(CLOCK_REALTIME,&t);
//    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
//    AutonomousCommandQuerry.push_back(help);
//}

//void MainWindow::on_pushButton_12_clicked()
//{
////    zX=ui->lineEdit_4->text().toDouble();
////    zY=ui->lineEdit_5->text().toDouble();
//    toleranciaUhla=2;
//    naviguj=true;
//}


void MainWindow::skeletonprocess()
{

    std::cout<<"init skeleton"<<std::endl;
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    ske_slen = sizeof(ske_si_other);
    if ((ske_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char ske_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    std::cout<<setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout)<<std::endl;
    std::cout<<setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene))<<std::endl;
#else
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23432);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23432);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    std::cout<<::bind(ske_s , (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) )<<std::endl;;
    char command=0x00;

    skeleton bbbk;
    double measure[225];
    while(stopall==1)
    {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char)*1800, 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1)
        {

        //    std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }


        memcpy(kostricka.joints,bbbk.joints,1800);
     updateSkeletonPicture=1;
  //      std::cout<<"doslo "<<ske_recv_len<<std::endl;
      //  continue;
        for(int i=0;i<75;i+=3)
        {
        //    std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}


void MainWindow::on_checkBox_clicked(bool checked)
{
    applyDelay=checked;
}

void MainWindow::imageViewer()
{
    cv::VideoCapture cap;
    cap.open("http://127.0.0.1:8889/stream.mjpg");
    cv::Mat frameBuf;
    while(1)
    {
        cap >> frameBuf;


        if(frameBuf.rows<=0)
        {
            std::cout<<"nefunguje"<<std::endl;
            continue;
        }

        if(applyDelay==true)
        {
            struct timespec t;
            CameraVector newcommand;
            frameBuf.copyTo(newcommand.data);
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            cameraQuerry.push_back(newcommand);
            for(int i=0;i<cameraQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-cameraQuerry[i].timestamp)).count()>(2.5*1000000000))
                {

                    cameraQuerry[i].data.copyTo(robotPicture);
                    cameraQuerry.erase(cameraQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


           frameBuf.copyTo(robotPicture);
        }
        frameBuf.copyTo(AutonomousrobotPicture);
        updateCameraPicture=1;

        update();
//        std::cout<<"vycital som"<<std::endl;
       // cv::imshow("client",frameBuf);
        cv::waitKey(1);
        QCoreApplication::processEvents();
    }
}




void MainWindow::on_stop_button_clicked() // EMERGENCY STOP BUTTON
{
//    cout << "Stop button: ";
    if(stop){
        stop = false;
//        cout << "true" << endl;
    }else{
        sendRobotCommand(ROBOT_STOP);
        stop = true;
//        cout << "false" << endl;
    }
}

void MainWindow::on_pushButton_4_clicked() // FORWARD
{
//    char tt=0x04;
//    sendRobotCommand(tt,3.14159/4);
    sendRobotCommand(ROBOT_VPRED, 250);
}

void MainWindow::on_pushButton_3_clicked() // BACKWARD
{
//    char tt=0x01;
//    sendRobotCommand(tt,250);
    sendRobotCommand(ROBOT_VZAD, -200);
}

void MainWindow::on_pushButton_2_clicked() // RIGHT
{
    sendRobotCommand(ROBOT_VPRAVO, -3.14159/4);
}

void MainWindow::on_pushButton_clicked() // LEFT
{
    sendRobotCommand(ROBOT_VLAVO, 3.14159/4);
}

