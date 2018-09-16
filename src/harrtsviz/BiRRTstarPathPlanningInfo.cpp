#include <sstream>
#include <cstdlib>
#include <list>
#include <QPixmap>
#include <QFile>

#include "topologyPathPlanning/harrtsviz/BiRRTstarPathPlanningInfo.hpp"

using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace harrts {

BIRRTstarPathPlanningInfo::BIRRTstarPathPlanningInfo() {
    mInfoFilename = "";
    mMapFilename = "";
    mMapFullpath = "";
    mObjectiveFile = "";
    m_start.setX(-1);
    m_start.setY(-1);
    m_goal.setX(-1);
    m_goal.setY(-1);

    mPathsOutput = "";
    mpFoundPaths.clear();
      
    mGrammarType = STRING_GRAMMAR_TYPE;
    mRunType = RUN_BOTH_TREES_TYPE;
   
    mMinDistEnabled = true;

    mMaxIterationNum = 1000;
    mSegmentLength = 5.0;
    mCostDistribution = NULL;

    mMapWidth = 0;
    mMapHeight = 0;
}

bool BIRRTstarPathPlanningInfo::getObstacleInfo( int** pp_obstacle_info ) {
    if( pp_obstacle_info==NULL ) {
        return false;
    }
    return getPixInfo( mMapFullpath, pp_obstacle_info );
}

bool BIRRTstarPathPlanningInfo::getCostDistribution( double** pp_cost_distribution ) {
    return getPixInfo( mObjectiveFile, pp_cost_distribution );
}

bool BIRRTstarPathPlanningInfo::getPixInfo( QString filename, double** pp_pix_info ) {
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

bool BIRRTstarPathPlanningInfo::getPixInfo(QString filename, int ** pp_pix_info) {
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


void BIRRTstarPathPlanningInfo::initFuncParam() {
    if( mMinDistEnabled == true ) {
        mpFunc = BIRRTstarPathPlanningInfo::calcDist;
        mCostDistribution = NULL;
    }
    else {
        mpFunc = BIRRTstarPathPlanningInfo::calcCost;
        if(mCostDistribution) {
            delete[] mCostDistribution;
            mCostDistribution = NULL;
        }
        mCostDistribution = new double*[mMapWidth];
        for(int i=0;i<mMapWidth;i++) {
            mCostDistribution[i] = new double[mMapHeight];
        }
        getCostDistribution( mCostDistribution );
    }
}

void BIRRTstarPathPlanningInfo::read( xmlNodePtr root ) {
    if( root->type == XML_ELEMENT_NODE ){
        xmlChar* tmp = xmlGetProp( root, ( const xmlChar* )( "map_filename" ) );
        if ( tmp != NULL ) {
            std::string map_filename = ( char * )( tmp ); 
            mMapFilename = QString::fromStdString( map_filename );
            xmlFree( tmp );
        } 
        tmp = xmlGetProp( root, ( const xmlChar* )( "map_fullpath" ) );
        if ( tmp != NULL ) {
            std::string map_fullpath = ( char * )( tmp );
            mMapFullpath = QString::fromStdString( map_fullpath ); 
            xmlFree( tmp );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "map_width" ) );
        if ( tmp != NULL ) {
            std::string map_width = ( char * )( tmp );
            mMapWidth = strtol( map_width.c_str(), NULL, 10 );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "height_width" ) );
        if ( tmp != NULL ) {
            std::string map_height = ( char * )( tmp );
            mMapHeight = strtol( map_height.c_str(), NULL, 10 );
        }
        int start_x_int = 0, start_y_int = 0;
        tmp = xmlGetProp( root, ( const xmlChar* )( "start_x" ) );
        if ( tmp != NULL ) {
            std::string start_x = ( char * )( tmp );
            start_x_int = strtol( start_x.c_str(), NULL, 10 );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "start_y" ) );
        if ( tmp != NULL ) {
            std::string start_y = ( char * )( tmp );
            start_y_int = strtol( start_y.c_str(), NULL, 10 );
        }
        m_start = QPoint( start_x_int, start_y_int );
        int goal_x_int = 0, goal_y_int = 0;
        tmp = xmlGetProp( root, ( const xmlChar* )( "goal_x" ) );
        if ( tmp != NULL ) {
            std::string goal_x = ( char * )( tmp );
            goal_x_int = strtol( goal_x.c_str(), NULL, 10 );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "goal_y" ) );
        if ( tmp != NULL ) {
            std::string goal_y = ( char * )( tmp );
            goal_y_int = strtol( goal_y.c_str(), NULL, 10 );
        }
        m_goal = QPoint( goal_x_int, goal_y_int );
        tmp = xmlGetProp( root, ( const xmlChar* )( "grammar_type" ) );
        if ( tmp != NULL ) {
            std::string grammar_type_str = ( char * )( tmp );
            mGrammarType = static_cast<grammar_type_t>( strtol( grammar_type_str.c_str(), NULL, 10 ) );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "run_type" ) );
        if ( tmp != NULL ) {
            std::string run_type_str = ( char * )( tmp );
            mRunType = static_cast<RRTree_run_type_t>( strtol( run_type_str.c_str(), NULL, 10 ) );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "min_dist_enabled" ) );
        if ( tmp != NULL ) {
            std::string min_dist_enabled = ( char * )( tmp );
            int min_dist_enabled_int = strtol( min_dist_enabled.c_str(), NULL, 10 );
            if (min_dist_enabled_int > 0) {
                mMinDistEnabled = true;
            } else {
                mMinDistEnabled = false;
            }
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "objective_file" ) );
        if ( tmp != NULL ) {
            std::string objective_file = ( char * )( tmp );
            mObjectiveFile = QString::fromStdString( objective_file );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "path_output_file" ) );
        if ( tmp != NULL ) {
            std::string paths_output = ( char * )( tmp );
            mPathsOutput = QString::fromStdString( paths_output );
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "max_iteration_num" ) );
        if ( tmp != NULL ) {
            std::string max_iter_num = ( char * )( tmp );
            mMaxIterationNum = strtol( max_iter_num.c_str(), NULL, 10);
        }
        tmp = xmlGetProp( root, ( const xmlChar* )( "segment_length" ) );
        if ( tmp != NULL ) {
            std::string seg_len = ( char * )( tmp );
            mSegmentLength = strtof( seg_len.c_str(), NULL );
        }
    }

}

void BIRRTstarPathPlanningInfo::write( xmlDocPtr doc, xmlNodePtr root ) const {
    
    xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "world" ), NULL );
    xmlNewProp( node, ( const xmlChar* )( "map_filename" ), ( const xmlChar* )( mMapFilename.toStdString().c_str() ) );
    xmlNewProp( node, ( const xmlChar* )( "map_fullpath" ), ( const xmlChar* )( mMapFullpath.toStdString().c_str() ) );
    std::stringstream width_str;
    width_str << mMapWidth;
    xmlNewProp( node, ( const xmlChar* )( "map_width" ), ( const xmlChar* )( width_str.str().c_str() ) );
    std::stringstream height_str;
    height_str << mMapHeight;
    xmlNewProp( node, ( const xmlChar* )( "map_height" ), ( const xmlChar* )( height_str.str().c_str() ) );
    std::stringstream start_x_str;
    start_x_str << m_start.x();
    xmlNewProp( node, ( const xmlChar* )( "start_x" ), ( const xmlChar* )( start_x_str.str().c_str() ) );
    std::stringstream start_y_str;
    start_y_str << m_start.y();
    xmlNewProp( node, ( const xmlChar* )( "start_y" ), ( const xmlChar* )( start_y_str.str().c_str() ) );
    std::stringstream goal_x_str;
    goal_x_str << m_goal.x();
    xmlNewProp( node, ( const xmlChar* )( "goal_x" ), ( const xmlChar* )( goal_x_str.str().c_str() ) );
    std::stringstream goal_y_str;
    goal_y_str << m_goal.y();
    xmlNewProp( node, ( const xmlChar* )( "goal_y" ), ( const xmlChar* )( goal_y_str.str().c_str() ) );
    std::stringstream grammar_t_str;
    grammar_t_str << static_cast<unsigned int>( mGrammarType );
    xmlNewProp( node, ( const xmlChar* )( "grammar_type"), ( const xmlChar* )( grammar_t_str.str().c_str() ) );    
    std::stringstream run_t_str;
    run_t_str << static_cast<unsigned int>( mRunType );
    xmlNewProp( node, ( const xmlChar* )( "run_type"), ( const xmlChar* )( run_t_str.str().c_str() ) );    
 
    std::string min_dist_enabled;
    if ( mMinDistEnabled ) {
        min_dist_enabled = "1";
    } else {
        min_dist_enabled = "0";
    }
    xmlNewProp( node, ( const xmlChar* )( "min_dist_enabled" ), ( const xmlChar* )( min_dist_enabled.c_str() ) );
    xmlNewProp( node, ( const xmlChar* )( "objective_file" ), ( const xmlChar* )( mObjectiveFile.toStdString().c_str() ) );
    xmlNewProp( node, ( const xmlChar* )( "path_output_file" ), ( const xmlChar* )( mPathsOutput.toStdString().c_str() ) );

    std::stringstream max_iter_num_str;
    max_iter_num_str << mMaxIterationNum;
    xmlNewProp( node, ( const xmlChar* )( "max_iteration_num" ), ( const xmlChar* )( max_iter_num_str.str().c_str() ) );
    std::stringstream seg_len_str;
    seg_len_str << mSegmentLength;
    xmlNewProp( node, ( const xmlChar* )( "segment_length" ), ( const xmlChar* )( seg_len_str.str().c_str() ) );
    
    xmlAddChild( root, node );
}

bool BIRRTstarPathPlanningInfo::saveToFile(QString filename) {
    
    xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
    xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );    xmlDocSetRootElement( doc, root );
    write( doc, root );
    xmlSaveFormatFileEnc( filename.toStdString().c_str(), doc, "UTF-8", 1 );
    xmlFreeDoc( doc );
    return true;
}

bool BIRRTstarPathPlanningInfo::loadFromFile(QString filename) {
    xmlDoc * doc = NULL;
    xmlNodePtr root = NULL;
    doc = xmlReadFile( filename.toStdString().c_str(), NULL, 0 );
    if ( doc != NULL ) {
        root = xmlDocGetRootElement( doc );
        if( root->type == XML_ELEMENT_NODE ) {
            xmlNodePtr l1 = NULL;
            for( l1 = root->children; l1; l1 = l1->next ) {
                if( l1->type == XML_ELEMENT_NODE ) { 
                    if( xmlStrcmp( l1->name, ( const xmlChar * )( "world" ) ) == 0 ){ 
                        read( l1 );
                    }
                }
            }
        }
    }
    return true;
}

void BIRRTstarPathPlanningInfo::loadPaths( std::vector<Path*> paths) {
    mpFoundPaths = paths;
}

bool BIRRTstarPathPlanningInfo::exportPaths(QString filename) {
    QFile file(filename);
    if( file.open(QIODevice::ReadWrite) ) {
        QTextStream stream( & file );
        for( std::vector<Path*>::iterator it = mpFoundPaths.begin();
             it != mpFoundPaths.end(); it++) {
            Path* p_path = (*it);
            // Save scores
            stream << p_path->mCost << "\n";
            stream << "\n";
            for(unsigned int i=0;i<p_path->mWaypoints.size();i++) {
                stream << p_path->mWaypoints[i][0] << " " << p_path->mWaypoints[i][1] << "\t";
            }
            stream << "\n";
            stream << "\n";
        }
        return true;
    }
    return false;
}

void BIRRTstarPathPlanningInfo::dumpCostDistribution( QString filename ) {
    QFile file(filename);
    if( file.open(QIODevice::ReadWrite) ) {
        QTextStream stream( & file );

        if( mCostDistribution ) {
            for(int i=0;i<mMapWidth;i++) {
                for(int j=0;j<mMapHeight;j++) {
                    stream << mCostDistribution[i][j] << " ";
                }
                stream << "\n";
            }
        }
    }
}

} // harrts

} // topologyPathPlanning
