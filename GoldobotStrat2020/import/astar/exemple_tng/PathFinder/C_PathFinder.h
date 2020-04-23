#ifndef C_PATH_FINDER_H
#define C_PATH_FINDER_H

#include <QMainWindow>
#include <QBrush>
#include <QCloseEvent>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QImage>
#include <Qpainter>
#include <QPen>
#include <QScrollBar>
#include <QTimer>
#include <QWidget>

#include "ui_C_PathFinder.h"

#include "astar.h"
#include "NeatoAPI/SerialComScanse/C_NApi_SerialComScanse.h"

class C_PathFinder : public QMainWindow
{
    Q_OBJECT

public:
    // Constructor
    explicit C_PathFinder(QWidget * ptParent, uint32_t mmPerPixel);

    // Destructor
    ~C_PathFinder();

public slots:
    void SLOT_MapUpdate(uint8_t * mapData_0, uint8_t * mapData_1, uint32_t xMin, uint32_t xMax, uint32_t yMin, uint32_t yMax);
    void SLOT_InitPos(double x__mm, double y__mm, double angle__rad);


signals:
    // Closing the window
    void SIG_ClosingWindow(void);
    void SIG_ExecuteCmd(C_NApi_SerialComScanse::enum_MvtCmd cmd, int speed, double param);

private:
    class MapScene : public QGraphicsScene
    {
    public:
        MapScene(QObject * parent = 0){m_ptParent = parent;}
        ~MapScene(){}

    protected:
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    private :
        QObject * m_ptParent;

    };

    // The UI of the object
    Ui::C_PathFinder * m_ptUi;

    double CONV_PIXEL_2_MM;
    double CONV_MM_2_PIXEL;

    // Robot position
    double m_robotPosX__pixel = 0.0;
    double m_robotPosY__pixel = 0.0;
    double m_robotAngle__rad = 0.0;

    AStar m_coreAstar;
    int m_mapSize__pixel = 0;
    bool m_needPathComputation = false;

    // Map view
    MapScene * m_ptMapGrScene;
    QGraphicsPixmapItem * m_ptMap;
    QImage m_coreSlamMap;
    QImage m_coreSlamMapToShow;

    void DrawRobotPath();

    void DriveTheRobot(void);

    double m_mapScale = 1.0;
    double m_mapViewCenterX = 0.0;
    double m_mapViewCenterY = 0.0;


    // Show the CoreSlam Map
    QTimer * m_ptCoreSlamShowMapTimer;

    // Show the CoreSlam Map
    QTimer * m_ptAutoDriveTimer;
    int m_currentPathPointId = 0;

    void SetValueOnMap(uint32_t value, int32_t x__pixel, int32_t y__pixel, int32_t margin__pixel = 0);

    // This function returns the angle in the same reference than the CoreSlam Map ref
    inline double ComputeAngle(double x0, double y0, double x1, double y1)
    {
        return atan2(y1 - y0, x1 - x0);
    }

    uint32_t m_tabConvSlamMapPixelToRgbPixel[256];


    // TOOLS CONSTANTS
    // ======================================================================================
    const double PI = 3.1415926535897932384626433832795;
    const double MINUS_PI = -PI;
    const double CONV_DEG_2_RAD = 0.01745329251994329576923690768489;
    const double CONV_RAD_2_DEG = 57.295779513082320876798154814105;
    const double PI_x_2 = PI * 2.0;
    const double HALF_PI = PI / 2.0;

private slots:
    // Closing the window event
    void closeEvent (QCloseEvent * ptEvent);

    void on_btn_ZoomDefaultMap_clicked();
    void on_btn_CenterMap_clicked();
    void on_btn_ZoomOutMap_clicked();
    void on_btn_ZoomInMap_clicked();
    void on_btn_MoveRightMap_clicked();
    void on_btn_MoveLeftMap_clicked();
    void on_btn_MoveDownMap_clicked();
    void on_btn_MoveUpMap_clicked();

    void SLOT_CoreSlamShowMap();
};

#endif // C_PATH_FINDER_H
