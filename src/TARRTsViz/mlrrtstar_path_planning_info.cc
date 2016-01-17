#include <sstream>
#include <cstdlib>
#include <list>
#include <QPixmap>
#include <QFile>

#include "mlrrtstar_path_planning_info.h"

using namespace homotopy;
using namespace mlrrts;

MLRRTstarPathPlanningInfo::MLRRTstarPathPlanningInfo() {
  m_info_filename = "";
  m_map_filename = "";
  m_map_fullpath = "";
  m_objective_file = "";
  m_start.setX(-1);
  m_start.setY(-1);
  m_goal.setX(-1);
  m_goal.setY(-1);

  m_paths_output = "";
  mp_found_paths.clear();
  
  m_min_dist_enabled = true;

  m_grammar_type = STRING_GRAMMAR_TYPE;
  m_max_iteration_num = 5000;
  m_segment_length = 5.0;
  m_cost_distribution = NULL;
  m_homotopic_enforcement = true;

  m_map_width = 0;
  m_map_height = 0;

  mp_obj = NULL;
}

MLRRTstarPathPlanningInfo::~MLRRTstarPathPlanningInfo() {
  if( mp_obj ) {
    delete mp_obj;
    mp_obj = NULL;
  }
}

bool MLRRTstarPathPlanningInfo::get_obstacle_info( int** pp_obstacle_info ) {
  if( pp_obstacle_info==NULL ) {
    return false;
  }
  return get_pix_info( m_map_fullpath, pp_obstacle_info );
}

bool MLRRTstarPathPlanningInfo::get_cost_distribution( double** pp_cost_distribution ) {
  return get_pix_info( m_objective_file, pp_cost_distribution );
}

bool MLRRTstarPathPlanningInfo::get_pix_info( QString filename, double** pp_pix_info ) {
  if( pp_pix_info==NULL ) {
    return false;
  }
  if( mp_obj ) {
    delete mp_obj;
    mp_obj = NULL;
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

bool MLRRTstarPathPlanningInfo::get_pix_info(QString filename, int ** pp_pix_info) {
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

void MLRRTstarPathPlanningInfo::init_obj_pixmap(){
  if( mp_obj ) {
    delete mp_obj;
    mp_obj = NULL;
  }
  
  mp_obj = new QPixmap( m_objective_file );   
    
}

void MLRRTstarPathPlanningInfo::init_func_param() {
  if( m_min_dist_enabled == true ) {
    mp_func = MLRRTstarPathPlanningInfo::calc_dist;
    m_cost_distribution = NULL;
  }
  else {
    mp_func = MLRRTstarPathPlanningInfo::calc_cost;
    if(m_cost_distribution) {
      delete[] m_cost_distribution;
      m_cost_distribution = NULL;
    }
    m_cost_distribution = new double*[m_map_width];
    for(int i=0;i<m_map_width;i++) {
      m_cost_distribution[i] = new double[m_map_height];
    }
    get_cost_distribution( m_cost_distribution );
  }
}

void MLRRTstarPathPlanningInfo::read( xmlNodePtr root ) {
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar* tmp = xmlGetProp( root, ( const xmlChar* )( "map_filename" ) );
    if ( tmp != NULL ) {
      std::string map_filename = ( char * )( tmp ); 
      m_map_filename = QString::fromStdString( map_filename );
      xmlFree( tmp );
    } 
    tmp = xmlGetProp( root, ( const xmlChar* )( "map_fullpath" ) );
    if ( tmp != NULL ) {
      std::string map_fullpath = ( char * )( tmp );
      m_map_fullpath = QString::fromStdString( map_fullpath ); 
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "map_width" ) );
    if ( tmp != NULL ) {
      std::string map_width = ( char * )( tmp );
      m_map_width = strtol( map_width.c_str(), NULL, 10 );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "height_width" ) );
    if ( tmp != NULL ) {
      std::string map_height = ( char * )( tmp );
      m_map_height = strtol( map_height.c_str(), NULL, 10 );
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
    
    tmp = xmlGetProp( root, ( const xmlChar* )( "min_dist_enabled" ) );
    if ( tmp != NULL ) {
      std::string min_dist_enabled = ( char * )( tmp );
      int min_dist_enabled_int = strtol( min_dist_enabled.c_str(), NULL, 10 );
      if (min_dist_enabled_int > 0) {
        m_min_dist_enabled = true;
      } else {
        m_min_dist_enabled = false;
      }
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "homotopic_enforcement" ) );
    if ( tmp != NULL ) {
      std::string homotopic_enforcement = ( char * )( tmp );
      int homotopic_enforcement_int = strtol( homotopic_enforcement.c_str(), NULL, 10 );
      if (homotopic_enforcement_int > 0) {
        m_homotopic_enforcement = true;
      } else {
        m_homotopic_enforcement = false;
      }
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "objective_file" ) );
    if ( tmp != NULL ) {
      std::string objective_file = ( char * )( tmp );
      m_objective_file = QString::fromStdString( objective_file );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "path_output_file" ) );
    if ( tmp != NULL ) {
      std::string paths_output = ( char * )( tmp );
      m_paths_output = QString::fromStdString( paths_output );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "max_iteration_num" ) );
    if ( tmp != NULL ) {
      std::string max_iter_num = ( char * )( tmp );
      m_max_iteration_num = strtol( max_iter_num.c_str(), NULL, 10);
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "segment_length" ) );
    if ( tmp != NULL ) {
      std::string seg_len = ( char * )( tmp );
      m_segment_length = strtof( seg_len.c_str(), NULL );
    }
  }

}

void MLRRTstarPathPlanningInfo::write( xmlDocPtr doc, xmlNodePtr root ) const {
    
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "world" ), NULL );
  xmlNewProp( node, ( const xmlChar* )( "map_filename" ), ( const xmlChar* )( m_map_filename.toStdString().c_str() ) );
  xmlNewProp( node, ( const xmlChar* )( "map_fullpath" ), ( const xmlChar* )( m_map_fullpath.toStdString().c_str() ) );
  std::stringstream width_str;
  width_str << m_map_width;
  xmlNewProp( node, ( const xmlChar* )( "map_width" ), ( const xmlChar* )( width_str.str().c_str() ) );
  std::stringstream height_str;
  height_str << m_map_height;
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

  std::string min_dist_enabled;
  if ( m_min_dist_enabled ) {
    min_dist_enabled = "1";
  } else {
    min_dist_enabled = "0";
  }
  xmlNewProp( node, ( const xmlChar* )( "min_dist_enabled" ), ( const xmlChar* )( min_dist_enabled.c_str() ) );
  std::string homotopic_enforcement;
  if ( m_homotopic_enforcement ) {
    homotopic_enforcement = "1";
  } else {
    homotopic_enforcement = "0";
  }
  xmlNewProp( node, ( const xmlChar* )( "min_dist_enabled" ), ( const xmlChar* )( min_dist_enabled.c_str() ) );
  xmlNewProp( node, ( const xmlChar* )( "objective_file" ), ( const xmlChar* )( m_objective_file.toStdString().c_str() ) );
  xmlNewProp( node, ( const xmlChar* )( "path_output_file" ), ( const xmlChar* )( m_paths_output.toStdString().c_str() ) );

  std::stringstream max_iter_num_str;
  max_iter_num_str << m_max_iteration_num;
  xmlNewProp( node, ( const xmlChar* )( "max_iteration_num" ), ( const xmlChar* )( max_iter_num_str.str().c_str() ) );
  std::stringstream seg_len_str;
  seg_len_str << m_segment_length;
  xmlNewProp( node, ( const xmlChar* )( "segment_length" ), ( const xmlChar* )( seg_len_str.str().c_str() ) );
    
  xmlAddChild( root, node );
}

bool MLRRTstarPathPlanningInfo::save_to_file(QString filename) {
    
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );    xmlDocSetRootElement( doc, root );
  write( doc, root );
  xmlSaveFormatFileEnc( filename.toStdString().c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return true;
}

bool MLRRTstarPathPlanningInfo::load_from_file(QString filename) {
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

void MLRRTstarPathPlanningInfo::load_paths( std::vector<Path*> paths) {
  mp_found_paths = paths;
}

bool MLRRTstarPathPlanningInfo::export_paths(QString filename) {
  QFile file(filename);
  if( file.open(QIODevice::ReadWrite) ) {
    QTextStream stream( & file );
    for( std::vector<Path*>::iterator it = mp_found_paths.begin();
         it != mp_found_paths.end(); it++) {
      Path* p_path = (*it);
      // Save scores
      stream << p_path->m_cost << "\n";
      stream << "\n";
      for(unsigned int i=0;i<p_path->m_way_points.size();i++) {
        stream << p_path->m_way_points[i][0] << " " << p_path->m_way_points[i][1] << "\t";
      }
      stream << "\n";
      stream << "\n";
    }
    return true;
  }
  return false;
}

void MLRRTstarPathPlanningInfo::dump_cost_distribution( QString filename ) {
  QFile file(filename);
  if( file.open(QIODevice::ReadWrite) ) {
    QTextStream stream( & file );

    if( m_cost_distribution ) {
      for(int i=0;i<m_map_width;i++) {
        for(int j=0;j<m_map_height;j++) {
          stream << m_cost_distribution[i][j] << " ";
        }
        stream << "\n";
      }
    }
  }
}
