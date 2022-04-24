#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifdef _WIN32
#include<windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include "unistd.h"
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#endif
#include<iostream>

#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include<algorithm>
#include<chrono>
#include "CKobuki.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rplidar.h"
#include <QThread>
#include <QKeyEvent>
#include <queue>

#define ROBOT_VPRED 0x01
#define ROBOT_VZAD 0x02
#define ROBOT_VLAVO 0x04
#define ROBOT_VPRAVO 0x03
#define ROBOT_STOP 0x00
#define ROBOT_ARC 0x05

#define SHORT_SIZE 65536
#define SHORT_HALF_SIZE 65536/2
#define MAX_UNSIGNED_SHORT 65535
//#define MAX_SIGNED_SHORT 32767
//#define MIN_SIGNED_SHORT -32768
#define MAX_GYRO 18000
#define MIN_GYRO -18000

#define MAX_ANGLE_SPEED 3.14159/2 // rad/s
#define MAX_SPEED 400       // mm/s

// DIRECTIONS
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4


static const char *klby[]={{"left_wrist"},{"left_thumb_cmc"},{"left_thumb_mcp"},{"left_thumb_ip"},{"left_thumb_tip"},{"left_index_cmc"},{"left_index_mcp"},{"left_index_ip"},{"left_index_tip"},{"left_middle_cmc"},{"left_middle_mcp"},{"left_middle_ip"},{"left_middle_tip"},{"left_ring_cmc"},{"left_ring_mcp"},{"left_ring_ip"},{"left_ringy_tip"},{"left_pinky_cmc"},{"left_pink_mcp"},{"left_pink_ip"},{"left_pink_tip"},{"right_wrist"},{"right_thumb_cmc"},{"right_thumb_mcp"},{"right_thumb_ip"},{"right_thumb_tip"},{"right_index_cmc"},{"right_index_mcp"},{"right_index_ip"},{"right_index_tip"},{"right_middle_cmc"},{"right_middle_mcp"},{"right_middle_ip"},{"right_middle_tip"},{"right_ring_cmc"},{"right_ring_mcp"},{"right_ring_ip"},{"right_ringy_tip"},{"right_pinky_cmc"},{"right_pink_mcp"},{"right_pink_ip"},{"right_pink_tip"},{"nose"},{"left_eye_in"},{"left_eye"},{"left_eye_out"},{"right_eye_in"},{"right_eye"},{"right_eye_out"},{"left_ear"},{"right_ear"},{"mounth_left"},{"mounth_right"},{"left_shoulder"},{"right_shoulder"},{"left_elbow"},{"right_elbow"},{"left_wrist"},{"right_wrist"},{"left_pinky"},{"right_pinky"},{"left_index"},{"right_index"},{"left_thumb"},{"right_thumb"},{"left_hip"},{"right_hip"},{"left_knee"},{"right_knee"},{"left_ankle"},{"right_ankle"},{"left_heel"},{"righ_heel"},{"left+foot_index"},{"right_foot_index"}};
enum jointnames
{
    left_wrist,
   left_thumb_cmc,
   left_thumb_mcp,
   left_thumb_ip,
   left_thumb_tip,
   left_index_cmc,
   left_index_mcp,
   left_index_ip,
   left_index_tip,
   left_middle_cmc,
   left_middle_mcp,
   left_middle_ip,
   left_middle_tip,
   left_ring_cmc,
   left_ring_mcp,
   left_ring_ip,
   left_ringy_tip,
   left_pinky_cmc,
   left_pink_mcp,
   left_pink_ip,
   left_pink_tip,
   right_wrist,
   right_thumb_cmc,
   right_thumb_mcp,
   right_thumb_ip,
   right_thumb_tip,
   right_index_cmc,
   right_index_mcp,
   right_index_ip,
   right_index_tip,
   right_middle_cmc,
   right_middle_mcp,
   right_middle_ip,
   right_middle_tip,
   right_ring_cmc,
   right_ring_mcp,
   right_ring_ip,
   right_ringy_tip,
   right_pinky_cmc,
   right_pink_mcp,
   right_pink_ip,
   right_pink_tip,
   nose,left_eye_in,
   left_eye,
   left_eye_out,
   right_eye_in,
   right_eye,
   right_eye_out,
   left_ear,
   right_ear,
   mounth_left,
   mounth_right,
   left_shoulder,
   right_shoulder,
   left_elbow,
   right_elbow,
   left_wrist_glob,
   right_wrist_glob,
   left_pinky,
   right_pinky,
   left_index,
   right_index,
   left_thumb,
   right_thumb,
   left_hip,
   right_hip,
   left_knee,
   right_knee,
   left_ankle,
   right_ankle,
   left_heel,
   righ_heel,
   left_foot_index,
   right_foot_index
};
typedef struct
{
    double x;
    double y;
    double z;
}klb;

typedef struct
{
    klb joints[75];
}skeleton;

typedef struct
{
    double x;
    double y;
}X_Y;

typedef struct
{
    int i;
    int j;
}IJ_idx;


typedef struct
{
    short diffEncLeft;
    short diffEncRight;
//    short diffGyroAngle;
    short initGyroAngle;

    unsigned short oldEncoderLeft;
    unsigned short oldEncoderRight;

    double x;
    double x_old;
    double y;
    double y_old;
    double phi;
    double phi_old;

    //Dist. control
    double setpoint; // 0.0
    double x_set;
    double y_set;
    short Kp_dist;
    unsigned short output;
    // Angle control
    double Kp_angle;
    double deathAngle;
    double pa1;
    double pa2;
    double outputAngle;

    short rampLimit;
    double rampLimitAngle;

}ControlData;

typedef struct
{
     std::chrono::steady_clock::time_point timestamp;
    int command;
    double speed;
    int radius;
}RobotCommand;

typedef struct
{
     std::chrono::time_point<std::chrono::steady_clock> timestamp;
    TKobukiData sens;
}RobotData;

typedef struct
{
    int commandType;//0 ziaden, 1 pohyb, 2 uhol
     int desiredDist;
     int actualDist;
     int desiredAngle;
     int actualAngle;
}AutonomousCommand;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    AutonomousCommand command;
}CommandVector;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    LaserMeasurement data;
}LidarVector;


typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat data;
}CameraVector;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    double robotX;
    double robotY;
    double robotFi;
    int globalcommand;


    int EncMax=65536;
    double PolohaX;
    double PolohaY;
    double Uhol;
    double GyroUhol;
    double GyroUholOld;
    double deltaUhol;
    int PomEncoderL;
    int PomEncoderR;
    int deltaEncL;
    int deltaEncR;
    double deltaVzdialenostL;
    double deltaVzdialenostR;
    bool prvyStart;
    double gyro;

std::string ipaddress;
    std::vector<RobotCommand> commandQuery;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //vlakno co cita robot

    bool naviguj;
    double zX;
    double zY;
    double toleranciaUhla;
    int dl;
    int stopall;
    std::thread robotthreadHandle;
 //   pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
std::thread laserthreadHandle;
  //  pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }



    std::thread skeletonthreadHandle;
      //  pthread_t laserthreadHandle; // handle na vlakno
        int skeletonthreadID;  // id vlakna
        static void *skeletonUDPVlakno(void *param)
        {
            std::cout<<"startujem ci co"<<std::endl;
            ((MainWindow*)param)->skeletonprocess();

            return 0;
        }
    QThread Imager;
    void imageViewer();
    void sendRobotCommand(char command,double speed=0,int radius=0);
    void autonomousRobotCommand(char command,double speed=0,int radius=0);

    void robotprocess();
    void laserprocess();
void skeletonprocess();
    void localrobot(TKobukiData &sens);
    void autonomousrobot(TKobukiData &sens);

    int locallaser(LaserMeasurement &laserData);
    int autonomouslaser(LaserMeasurement &laserData);

    void paintThisLidar(LaserMeasurement &laserData);
    LaserMeasurement paintLaserData;
    int updateLaserPicture;
    int updateCameraPicture;
    int updateSkeletonPicture;
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;


    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int ske_s,  ske_recv_len;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;

#ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
#else
         unsigned int rob_slen;
         unsigned int las_slen;
         unsigned int ske_slen;
#endif
 CKobuki robot;
 TKobukiData sens;
    QTimer *timer;
    std::vector<RobotData> sensorQuerry;
    std::vector<LidarVector> lidarQuerry;
    std::vector<CameraVector> cameraQuerry;
    std::vector< CommandVector> AutonomousCommandQuerry;
    cv::Mat robotPicture;
    cv::Mat AutonomousrobotPicture;

    skeleton kostricka;

    ControlData controlData;
    // FIFO pole
    vector<X_Y> fifo_array;

//    TKobukiData robotData;

    // Mapa
    unsigned short mapHeight; // metrov
    unsigned short mapWidth; // metrov
    double step;
    short gridRows;
    short gridCols;
    short mapSampleStep;
    bool readLaserToMap;
    bool ending = true;

//    short grid[(int)(6/0.1)][(int)(6/0.1)];
//    short map[(int)(6/0.1)][(int)(6/0.1)];

    short grid[(int)(12/0.05)][(int)(12/0.05)];
    short map[(int)(12/0.05)][(int)(12/0.05)];

    QPoint qpoint;
    QRect rect;

    bool doControl = false;

private slots:

    void on_pushButton_3_clicked();

//    void on_pushButton_7_clicked();

//    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

//    void on_pushButton_6_clicked();

    void on_pushButton_clicked();
    void robotexec();

//    void on_pushButton_8_clicked();

//    void on_pushButton_10_clicked();

//    void on_pushButton_11_clicked();

//    void on_pushButton_9_clicked();

//    void on_pushButton_12_clicked();

//    void on_pushButton_13_clicked();

    void on_checkBox_clicked(bool checked);

    void on_stop_button_clicked();

    void on_pushButton_2_clicked();


    //Moje
    void initialValues();
    void calcDifferences();
    void calcLocalisation();
    void regulateAngle(double error_angle);
    void regulatePosition(double error_distance);

    void compMap(LaserMeasurement laserData);

    IJ_idx placeInGrid(X_Y xyg);
    X_Y getXYFromMapIdx(IJ_idx idxs);

    X_Y pixelToCoords(int px, int py);
    IJ_idx coordsToPixels(double x, double y);

private:

    bool showCamera;
    bool showLidar;
    bool showSkeleton;

    bool applyDelay;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* event);
    void mousePressEvent(QMouseEvent* event);
};

#endif // MAINWINDOW_H
