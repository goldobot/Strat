#include "C_PathFinder.h"

C_PathFinder::C_PathFinder(QWidget * ptParent, uint32_t mmPerPixel) :
    QMainWindow(ptParent),
    m_ptUi(new Ui::C_PathFinder)
{
    CONV_PIXEL_2_MM = (double)mmPerPixel;
    CONV_MM_2_PIXEL = 1.0 / CONV_PIXEL_2_MM;

    // Configure the HMI
    m_ptUi->setupUi(this);
    m_mapSize__pixel = (int32_t)((double)(GLOBAL_MAP_SIDE_SIZE__MM) * CONV_MM_2_PIXEL) + 1;

    m_coreAstar.setMatrix(m_mapSize__pixel, m_mapSize__pixel);

    // Create the scene to show the scan
    m_ptMapGrScene = new MapScene(this);
    // Associate the scene to the graphic view
    m_ptUi->gv_Map->setScene(m_ptMapGrScene);

    m_coreSlamMap = QImage(m_mapSize__pixel, m_mapSize__pixel , QImage::Format_RGB32);
    m_coreSlamMap.fill(QColor(0,0,0));

    m_coreSlamMapToShow = QImage(m_mapSize__pixel, m_mapSize__pixel , QImage::Format_RGB32);
    m_coreSlamMapToShow.fill(QColor(255,255,255));

    // Convert the grayscale from 8bits to 32 bits
    for (uint32_t i = 0; i < 256; i++)
    {
        /*
        if(i < 100) m_tabConvSlamMapPixelToRgbPixel[i] = 0xFF000000;
        else m_tabConvSlamMapPixelToRgbPixel[i] = (0xFF000000 | (i << 16) | (i << 8) | i);
        */

        m_tabConvSlamMapPixelToRgbPixel[i] = (0xFF000000 | (i << 16) | (i << 8) | i);
    }


    // Create a dummy image to activate centering
    m_ptMap = m_ptMapGrScene->addPixmap(QPixmap::fromImage(m_coreSlamMapToShow));

    on_btn_CenterMap_clicked();

    // Set default zoom
    on_btn_ZoomDefaultMap_clicked();

    // Show map timer
    m_ptCoreSlamShowMapTimer = new QTimer(this);
    connect(m_ptCoreSlamShowMapTimer, &QTimer::timeout, this, &C_PathFinder::SLOT_CoreSlamShowMap);
    m_ptCoreSlamShowMapTimer->start(250);

    // Auto drive timer
    m_ptAutoDriveTimer = new QTimer(this);
    connect(m_ptAutoDriveTimer, &QTimer::timeout, this, &C_PathFinder::DriveTheRobot);
    m_ptAutoDriveTimer->start(250);
}

C_PathFinder::~C_PathFinder()
{
    delete m_ptUi;
}

void C_PathFinder::closeEvent (QCloseEvent * ptEvent)
{
    Q_UNUSED(ptEvent);

    // Send a signal to the parent
    emit SIG_ClosingWindow();
}

void C_PathFinder::DrawRobotPath()
{
    list<pair<UINT, UINT> > path = m_coreAstar.getPath(AStarPathType::raw);
    if(path.size() > 0)
    {
        list<pair<UINT, UINT> >::iterator pathIt;

        for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
        {
            uint32_t * scan = (uint32_t *) m_coreSlamMapToShow.scanLine(pathIt->second) + pathIt->first;
            *scan = 0xFFFFAA00;
        }
    }

    path = m_coreAstar.getPath(AStarPathType::smooth);
    if(path.size() > 0)
    {
        list<pair<UINT, UINT> >::iterator pathIt;

        for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
        {
            uint32_t * scan = (uint32_t *) m_coreSlamMapToShow.scanLine(pathIt->second) + pathIt->first;
            *scan = 0xFF0077FF;
        }
    }

    /*
    path = m_coreAstar.getPath(AStarPathType::smoothMargin);
    if(path.size() > 0)
    {
        list<pair<UINT, UINT> >::iterator pathIt;

        for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
        {
            uint32_t * scan = (uint32_t *) m_coreSlamMapToShow.scanLine(pathIt->second) + pathIt->first;
            *scan = 0xFF00FF00;
        }
    }
    */
}

void C_PathFinder::DriveTheRobot(void)
{
    m_ptAutoDriveTimer->stop();

    if(m_needPathComputation)
    {
        bool isNewPath = false;

        // Read the path to do
        list<pair<UINT, UINT> > path = m_coreAstar.getPathOnlyIfNeed(true, &isNewPath);

        if(isNewPath)
        {
            m_currentPathPointId = 0;
        }

        m_ptUi->lbl_nextPosIndex->setText(QString("Next target index : %1").arg(m_currentPathPointId) + QString(" (%1 targets)").arg(path.size()));

        if(path.size() > m_currentPathPointId)
        {
            list<pair<UINT, UINT>>::iterator pathIt = std::next(path.begin(), m_currentPathPointId);

            // Add 0.5 pixel to target the center of the pixel
            double targetX__pixel = ((double)pathIt->first) + 0.5;
            double targetY__pixel = ((double)pathIt->second) + 0.5;

            double angle = ComputeAngle(m_robotPosX__pixel, m_robotPosY__pixel, targetX__pixel, targetY__pixel);

            m_ptUi->lbl_nextPosX->setText(QString("Next X : %1 mm").arg(targetX__pixel * CONV_PIXEL_2_MM));
            m_ptUi->lbl_nextPosY->setText(QString("Next Y : %1 mm").arg(targetY__pixel * CONV_PIXEL_2_MM));
            m_ptUi->lbl_nextPosAngle->setText(QString("Next Angle : %1 deg").arg(angle * CONV_RAD_2_DEG));

            double dx = targetX__pixel - m_robotPosX__pixel;
            double dy = targetY__pixel - m_robotPosY__pixel;
            double distance__mm = sqrt(dx*dx+dy*dy) * CONV_PIXEL_2_MM;

            // Position is not good
            if(distance__mm >= 100.0)
            {
                int speed = 1;
                if(distance__mm > 500.0) speed = 3;
                else if(distance__mm > 300.0) speed = 2;

                // Compute the action to do to get the robot to the right angle
                angle -= m_robotAngle__rad;
                while (angle > PI) angle -= PI_x_2;
                while (angle < MINUS_PI) angle += PI_x_2;

                // Convert to deg
                angle *= CONV_RAD_2_DEG;

                // The angle is not OK, drive the robot
                if(abs(angle) > 15.0)
                {
                    if(angle > 0.0)
                    {
                        emit SIG_ExecuteCmd(C_NApi_SerialComScanse::TURN_RIGHT, 1, angle);
                    }
                    else
                    {
                        emit SIG_ExecuteCmd(C_NApi_SerialComScanse::TURN_LEFT, 1, -angle);
                    }
                }
                else if(abs(angle) > 5.0)
                {
                    if(angle > 0.0)
                    {
                        emit SIG_ExecuteCmd(C_NApi_SerialComScanse::FORWARD_RIGHT, speed, distance__mm);
                    }
                    else
                    {
                        emit SIG_ExecuteCmd(C_NApi_SerialComScanse::FORWARD_LEFT, speed, distance__mm);
                    }
                }
                else
                {
                    emit SIG_ExecuteCmd(C_NApi_SerialComScanse::FORWARD, speed, distance__mm);
                }
            }
            else
            {
                // To be sure
                emit SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);
                emit SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);

                // Go to next waypoint, if any
                if(path.size() > (m_currentPathPointId +1))
                {
                    m_currentPathPointId++;
                }
            }

            m_ptAutoDriveTimer->start(250);
            return;
        }
        else
        {
            // To be sure
            emit SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);
            emit SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);

            m_ptUi->lbl_nextPosX->setText(QString("Next X : - cm"));
            m_ptUi->lbl_nextPosY->setText(QString("Next Y : - cm"));
            m_ptUi->lbl_nextPosAngle->setText(QString("Next Angle : - deg"));
        }

        m_ptAutoDriveTimer->start(250);
        return;
    }

    m_ptUi->lbl_nextPosIndex->setText(QString("Next target index : - / -"));
    m_ptUi->lbl_nextPosX->setText(QString("Next X : - cm"));
    m_ptUi->lbl_nextPosY->setText(QString("Next Y : - cm"));
    m_ptUi->lbl_nextPosAngle->setText(QString("Next Angle : - deg"));

    m_ptAutoDriveTimer->start(250);
    return;
}


void C_PathFinder::on_btn_ZoomDefaultMap_clicked()
{
    m_ptUi->gv_Map->resetMatrix();
    m_ptUi->gv_Map->scale(3, 3);
}

void C_PathFinder::on_btn_CenterMap_clicked()
{
    m_mapViewCenterX = m_ptUi->gv_Map->sceneRect().center().x();
    m_mapViewCenterY = m_ptUi->gv_Map->sceneRect().center().y();
    m_ptUi->gv_Map->centerOn(m_mapViewCenterX, m_mapViewCenterY);
}

void C_PathFinder::on_btn_ZoomOutMap_clicked()
{
    m_ptUi->gv_Map->scale(0.8, 0.8);
}

void C_PathFinder::on_btn_ZoomInMap_clicked()
{
    m_ptUi->gv_Map->scale(1.2, 1.2);
}

void C_PathFinder::on_btn_MoveRightMap_clicked()
{
    m_mapViewCenterX -= 2.0;
    m_ptUi->gv_Map->centerOn(m_mapViewCenterX, m_mapViewCenterY);
}

void C_PathFinder::on_btn_MoveLeftMap_clicked()
{
    m_mapViewCenterX += 2.0;
    m_ptUi->gv_Map->centerOn(m_mapViewCenterX, m_mapViewCenterY);
}

void C_PathFinder::on_btn_MoveDownMap_clicked()
{
    m_mapViewCenterY -= 2.0;
    m_ptUi->gv_Map->centerOn(m_mapViewCenterX, m_mapViewCenterY);
}

void C_PathFinder::on_btn_MoveUpMap_clicked()
{
    m_mapViewCenterY += 2.0;
    m_ptUi->gv_Map->centerOn(m_mapViewCenterX, m_mapViewCenterY);
}

void C_PathFinder::SLOT_MapUpdate(uint8_t * mapData_0, uint8_t * mapData_1, uint32_t xMin, uint32_t xMax, uint32_t yMin, uint32_t yMax)
{
    int screenWidth = m_coreSlamMap.size().width();
    int offsetY = yMin * screenWidth;

    // First pass, re copy map data
    for(uint32_t y = yMin; y < yMax; y++)
    {
        uint32_t * scan = (uint32_t *) m_coreSlamMap.scanLine(y) + xMin;
        uint8_t * ptSlamMap = mapData_1 + offsetY + xMin;

        for(uint32_t x = xMin; x < xMax; x++)
        {
            *scan = m_tabConvSlamMapPixelToRgbPixel[*ptSlamMap];

            ptSlamMap++;
            scan++;
        }

        offsetY += screenWidth;
    }


    // Now do an extra pass to add margin
    QPainter qPainter(&m_coreSlamMap);
    qPainter.setBrush(QColor(0,0,0));
    qPainter.setPen(QColor(0,0,0));

    offsetY = yMin * screenWidth;

    for(uint32_t y = yMin; y < yMax; y++)
    {
        uint8_t * ptSlamMap = mapData_0 + offsetY + xMin;

        for(uint32_t x = xMin; x < xMax; x++)
        {
            if((*ptSlamMap) <= 128)
            {
                qPainter.drawEllipse(x - 4, y - 4, 8, 8);
            }

            ptSlamMap++;
        }

        offsetY += screenWidth;
    }


    // Update the internal Astar data
    for(uint32_t y = yMin; y < yMax; y++)
    {
        uint32_t * scan = (uint32_t *)m_coreSlamMap.scanLine(y) + xMin;

        for(uint32_t x = xMin; x < xMax; x++)
        {
            if((*scan) == 0xFF000000)
            {
                m_coreAstar.setWall(x, y);
            }
            else
            {
                m_coreAstar.setWay(x, y, 255 - ((*scan) & 0x000000FF));
            }

            scan++;
        }
    }

    // The robot pos is of course free space
     m_coreAstar.setWay((int)m_robotPosX__pixel, (int)m_robotPosY__pixel, 1);
}

void C_PathFinder::SLOT_InitPos(double x__mm, double y__mm, double angle__rad)
{
    m_robotPosX__pixel = x__mm * CONV_MM_2_PIXEL;
    m_robotPosY__pixel = y__mm * CONV_MM_2_PIXEL;
    m_robotAngle__rad = angle__rad;

    m_ptUi->lbl_coreSlamPosX->setText(QString("Cur X : %1 mm").arg(x__mm));
    m_ptUi->lbl_coreSlamPosY->setText(QString("Cur Y : %1 mm").arg(y__mm));
    m_ptUi->lbl_coreSlamPosAngle->setText(QString("Cur Angle : %1 deg").arg(m_robotAngle__rad * CONV_RAD_2_DEG));
}


// Set the given point as obstable on the map
// And the surrounding point also as obstacle
void C_PathFinder::SetValueOnMap(uint32_t value, int32_t x__pixel, int32_t y__pixel, int32_t margin__pixel)
{
    QPainter qPainter(&m_coreSlamMapToShow);
    qPainter.setBrush(Qt::red);
    qPainter.setPen(Qt::red);
    qPainter.drawEllipse(x__pixel - margin__pixel,y__pixel -margin__pixel, margin__pixel * 2 ,margin__pixel * 2);

    return;

    // Limit of the zone of obstacle
    int xmin = x__pixel - margin__pixel;
    int xmax = x__pixel + margin__pixel;
    int ymin = y__pixel - margin__pixel;
    int ymax = y__pixel + margin__pixel;

    // Protection
    if(xmin < 0) xmin = 0;
    else if(xmin > (m_mapSize__pixel - 1)) xmin = m_mapSize__pixel - 1;

    if(xmax < 0) xmax = 0;
    else if(xmax > (m_mapSize__pixel - 1)) xmax = m_mapSize__pixel - 1;

    if(ymin < 0) ymin = 0;
    else if(ymin > (m_mapSize__pixel - 1)) ymin = m_mapSize__pixel - 1;

    if(ymax < 0) ymax = 0;
    else if(ymax > (m_mapSize__pixel - 1)) ymax = m_mapSize__pixel - 1;

    // Now set the obstacle point
    for(int32_t y = ymin; y <= ymax; y++)
    {
        uint32_t * scan = ((uint32_t *)m_coreSlamMapToShow.scanLine(y)) + xmin;

        for(int32_t x = xmin; x <= xmax; x++)
        {
            (*scan) = value;
            scan++;
        }
    }

}

void C_PathFinder::SLOT_CoreSlamShowMap()
{
    m_ptCoreSlamShowMapTimer->stop();

    // Copy the entire map for visual purpose
    m_coreSlamMapToShow = m_coreSlamMap.copy();

    /*
    // Update the map
    // Drawer all obstacle space
    for(int32_t y = 0; y < m_mapSize__pixel; y++)
    {
        uint32_t * scanSrc = (uint32_t *)m_coreSlamMap.scanLine(y);
        uint32_t * scanDst = (uint32_t *)m_coreSlamMapToShow.scanLine(y);

        for(int32_t x = 0; x < m_mapSize__pixel; x++)
        {
            if((*scanSrc) == 0xFF000000) SetValueOnMap(0xFF000000, x, y, 4);

            scanSrc++;
            scanDst++;
        }
    }
    */

    /*

    // Update the map
    // Drawer all obstacle space
    for(int32_t y = 0; y < m_mapSize__pixel; y++)
    {
        uint32_t * scanSrc = (uint32_t *)m_coreSlamMap.scanLine(y);
        uint32_t * scanDst = (uint32_t *)m_coreSlamMapToShow.scanLine(y);

        for(int32_t x = 0; x < m_mapSize__pixel; x++)
        {
            if((*scanSrc) <= 0xFF808080)
                SetValueOnMap(0xFF000000, x, y, 4);

            scanSrc++;
            scanDst++;
        }
    }
*/

    // The robot pos is of course free space
    //SetValueOnMap(0xFF000000, (int)m_robotPosX__pixel, (int)m_robotPosY__pixel, 1);

    if(m_needPathComputation)
    {
        // The path is not anymore valid
        if(!m_coreAstar.IsPathStillValid())
        {
            // Need to recompute a new path from the current position
            m_coreAstar.setStart((int)m_robotPosX__pixel, (int)m_robotPosY__pixel);
        }

        // Compute the way
        DrawRobotPath();
    }

    // Show the current position
    m_coreSlamMapToShow.setPixel((int)m_robotPosX__pixel, (int)m_robotPosY__pixel, 0xFFFF00FF);

    // Show the map
    m_ptMap->setPixmap(QPixmap::fromImage(m_coreSlamMapToShow));

    // Center the map on the robot if needed
    if(m_ptUi->chkB_FollowRobot->isChecked())
    {
        m_ptUi->gv_Map->centerOn((int)m_robotPosX__pixel, (int)m_robotPosY__pixel);
    }

    // Refreshing timer
    m_ptCoreSlamShowMapTimer->start(250);
}


void C_PathFinder::MapScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    C_PathFinder * ptParent = (C_PathFinder *)m_ptParent;

    QPointF clickedPos;

    switch(event->button())
    {
    case Qt::LeftButton:
        clickedPos = event->buttonDownScenePos(Qt::LeftButton);
        ptParent->m_coreAstar.setStart((int)(ptParent->m_robotPosX__pixel), (int)(ptParent->m_robotPosY__pixel));
        ptParent->m_coreAstar.setEnd(clickedPos.x(), clickedPos.y());
        ptParent->m_needPathComputation = true;
        break;

    case Qt::RightButton:
        ptParent->m_needPathComputation = false;
        emit ptParent->SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);
        emit ptParent->SIG_ExecuteCmd(C_NApi_SerialComScanse::STOP, 0, 0);
        break;

    default:
        break;
    }
}


#include "moc_C_PathFinder.cpp"

