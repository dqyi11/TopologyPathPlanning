#include <QPixmap>
#include <QJsonArray>
#include <QFile>
#include <QJsonDocument>
#include <list>

#include "path_planning_info.h"

PathPlanningInfo::PathPlanningInfo() {
    m_info_filename = "";
    m_map_filename = "";
    m_map_fullpath = "";
    m_objective_file = "";
    m_start.setX(-1);
    m_start.setY(-1);
    m_goal.setX(-1);
    m_goal.setY(-1);

    m_paths_output = "";
    mp_found_path = NULL;

    m_min_dist_enabled = false;

    m_max_iteration_num = 100;
    m_segment_length = 5.0;
    mCostDistribution = NULL;

    m_map_width = 0;
    m_map_height = 0;
}

bool PathPlanningInfo::get_obstacle_info( int** pp_obstacle_info ) {
    if( pp_obstacle_info==NULL ) {
        return false;
    }
    return get_pix_info( m_map_fullpath, pp_obstacle_info );
}

bool PathPlanningInfo::get_cost_distribution( double** pp_cost_distribution ) {
    return get_pix_info( m_objective_file, pp_cost_distribution );
}

bool PathPlanningInfo::get_pix_info( QString filename, double** pp_pix_info ) {
    if( pp_pix_info==NULL ) {
        return false;
    }
    QPixmap map(filename);
    QImage gray_img = map.toImage();
    int width = map.width();
    int height = map.height();

    for(int i=0;i<width;i++) {
        for(int j=0;j<height;j++) {
            QRgb col = gray_img.pixel(i,j);
            int g_val = qGray(col);
            if( g_val < 0 || g_val > 255 ) {
                qWarning() << "gray value out of range";
            }
            pp_pix_info[i][j] = (double)g_val/255.0;
        }
    }
    return true;
}

bool PathPlanningInfo::get_pix_info(QString filename, int ** pp_pix_info) {
    if( pp_pix_info==NULL ) {
        return false;
    }
    QPixmap map(filename);
    QImage gray_img = map.toImage();
    int width = map.width();
    int height = map.height();

    for(int i=0;i<width;i++) {
        for(int j=0;j<height;j++) {
            QRgb col = gray_img.pixel(i,j);
            int g_val = qGray(col);
            if( g_val < 0 || g_val > 255 ) {
                qWarning() << "gray value out of range";
            }
            pp_pix_info[i][j] = g_val;
        }
    }
    return true;
}


void PathPlanningInfo::init_func_param() {
    if( m_min_dist_enabled == true ) {
        mp_func = PathPlanningInfo::calc_dist;
        mCostDistribution = NULL;
    }
    else {
        mp_func = PathPlanningInfo::calc_cost;
        if(mCostDistribution) {
            delete[] mCostDistribution;
            mCostDistribution = NULL;
        }
        mCostDistribution = new double*[m_map_width];
        for(int i=0;i<m_map_width;i++) {
            mCostDistribution[i] = new double[m_map_height];
        }
        get_cost_distribution( mCostDistribution );
    }
}

void PathPlanningInfo::read( const QJsonObject &json ) {
    //m_info_filename;
    m_map_filename = json["mapFilename"].toString();
    m_map_fullpath = json["mapFullpath"].toString();
    m_map_width = json["mapWidth"].toInt();
    m_map_height = json["mapHeight"].toInt();
    m_start = QPoint(json["startX"].toInt(), json["startY"].toInt());
    m_goal = QPoint(json["goalX"].toInt(), json["goalY"].toInt());

    m_min_dist_enabled = json["minDistEnabled"].toBool();
    m_objective_file = json["objectiveFile"].toString();
    m_paths_output = json["pathOutputFile"].toString();

    m_max_iteration_num = json["maxIterationNum"].toInt();
    m_segment_length = json["segmentLength"].toDouble();
}

void PathPlanningInfo::write(QJsonObject &json) const {
    json["mapFilename"] = m_map_filename;
    json["mapFullpath"] = m_map_fullpath;
    json["mapWidth"]    = m_map_width;
    json["mapHeight"]   = m_map_height;

    json["startX"] = m_start.x();
    json["startY"] = m_start.y();
    json["goalX"]  = m_goal.x();
    json["goalY"]  = m_goal.y();

    json["minDistEnabled"] = m_min_dist_enabled;
    json["objectiveFile"]  = m_objective_file;
    json["pathOutputFile"] = m_paths_output;

    json["maxIterationNum"] = m_max_iteration_num;
    json["segmentLength"]   = m_segment_length;
}

bool PathPlanningInfo::save_to_file(QString filename) {
    QFile saveFile(filename);

    if( false==saveFile.open(QIODevice::WriteOnly) ) {
        qWarning("Couldn't open file.");
        return false;
    }

    QJsonObject infoObject;
    write(infoObject);
    QJsonDocument saveDoc(infoObject);
    saveFile.write(saveDoc.toJson());
    return true;
}

bool PathPlanningInfo::load_from_file(QString filename) {
    QFile load_file(filename);

    if( false==load_file.open(QIODevice::ReadOnly) ) {
        qWarning("Couldn't open file.");
        return false;
    }

    QByteArray saveData = load_file.readAll();
    QJsonDocument loadDoc = QJsonDocument::fromJson(saveData);
    read(loadDoc.object());
    return true;
}

void PathPlanningInfo::load_path(Path* path) {
    mp_found_path = path;
}

bool PathPlanningInfo::export_path(QString filename) {
    QFile file(filename);
    if( file.open(QIODevice::ReadWrite) ) {
        QTextStream stream( & file );

        if( mp_found_path ) {
            // Save scores
            stream << mp_found_path->m_cost << "\n";
            stream << "\n";
            for(int i=0;i<mp_found_path->m_way_points.size();i++) {
                stream << mp_found_path->m_way_points[i][0] << " " << mp_found_path->m_way_points[i][1] << "\t";
            }
            stream << "\n";
        }
        return true;
    }
    return false;
}

void PathPlanningInfo::dump_cost_distribution( QString filename ) {
    QFile file(filename);
    if( file.open(QIODevice::ReadWrite) ) {
        QTextStream stream( & file );

        if( mCostDistribution ) {
            for(int i=0;i<m_map_width;i++) {
                for(int j=0;j<m_map_height;j++) {
                    stream << mCostDistribution[i][j] << " ";
                }
                stream << "\n";
            }
        }
    }
}
