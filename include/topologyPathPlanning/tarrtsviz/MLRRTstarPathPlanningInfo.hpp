#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARPATHPLANNINGINFO_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARPATHPLANNINGINFO_HPP

#include <math.h>
#include <list>
#include <vector>
#include <libxml/tree.h>
#include <QString>
#include <QPoint>
#include <QPixmap>
#include <QDebug>

#include "topologyPathPlanning/tarrts/MLRRTstar.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  class MLRRTstarPathPlanningInfo {
  public:
    MLRRTstarPathPlanningInfo();
    virtual ~MLRRTstarPathPlanningInfo();

    bool getObstacleInfo( int** pp_obstacle_info );
    bool getCostDistribution( double** pp_cost_distribution );

    bool getPixInfo( QString filename, double** pp_pix_info );
    bool getPixInfo( QString filename, int** pp_pix_info );
    void initFuncParam();

    void dumpCostDistribution( QString filename );

    bool saveToFile( QString filename );
    bool loadFromFile( QString filename );

    void read( xmlNodePtr root );
    void write( xmlDocPtr doc, xmlNodePtr root ) const;

    void loadPaths( std::vector<Path*> paths );
    bool exportPaths( QString filename );
 
    void initObjPixmap();

    static double calcDist( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
      double dist = 0.0;
      if (pos_a == pos_b) {
        return dist;
      }
      double delta_x = fabs(pos_a[0]-pos_b[0]);
      double delta_y = fabs(pos_a[1]-pos_b[1]);
      dist = sqrt(delta_x*delta_x+delta_y*delta_y);

      if(dist < 0.0) {
        qWarning() << "Dist negative " << dist ;
      }
      return dist;
    }

    static double calc_cost( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
      double cost = 0.0;
      MLRRTstar* rrts = (MLRRTstar*)tree;
      if ( pos_a == pos_b ) {
        return cost;
      }
      if( pp_distribution == NULL ) {
        return cost;
      }

      float x1 = pos_a[0];
      float y1 = pos_a[1];
      float x2 = pos_b[0];
      float y2 = pos_b[1];

      const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
      if (steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
      }

      if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
      }

      const float dx = x2 - x1;
      const float dy = fabs(y2 - y1);

      float error = dx / 2.0f;
      const int ystep = (y1 < y2) ? 1 : -1;
      int y = (int)y1;

      const int maxX = (int)x2;

      for(int x=(int)x1; x<maxX; x++) {
        if(steep) {
          if (y>=0 && y<rrts->getSamplingWidth() && x>=0 && x<rrts->getSamplingHeight()) {
            cost += pp_distribution[y][x];
          }
        }
        else {
          if (x>=0 && x<rrts->getSamplingWidth() && y>=0 && y<rrts->getSamplingHeight()) {
            cost += pp_distribution[x][y];
          }
        }

        error -= dy;
        if(error < 0) {
          y += ystep;
          error += dx;
        }
      }
      return cost;
    }

    /* Member variables */
    QString mInfoFilename;
    QString mMapFilename;
    QString mMapFullpath;
    int mMapWidth;
    int mMapHeight;

    QPoint mStart;
    QPoint mGoal;

    QString mPathsOutput;
    bool mMinDistEnabled;
    QString mObjectiveFile;

    COST_FUNC_PTR mpFunc;
    double** mCostDistribution;

    QPixmap* mp_obj;

    homotopy::grammar_type_t mGrammarType;

    int    mMaxIterationNum;
    double mSegmentLength;
    bool   mHomotopicEnforcement;

    std::vector<Path*> mpFoundPaths;
  };

} // tarrts

} // topologyPathPlanning

#endif //  TOPOLOGYPATHPLANNING_MLRRTSTARPATHPLANNINGINFO_HPP
