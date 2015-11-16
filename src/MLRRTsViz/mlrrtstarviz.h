#ifndef MLRRTSTAR_VIZ_H_
#define MLRRTSTAR_VIZ_H_

#include <QLabel>

#include "mlrrtstar.h"
#include "path_planning_info.h"

namespace mlrrts{

  class MLRRTstarViz : public QLabel
  {
    Q_OBJECT
  public:
    explicit MLRRTstarViz(QWidget* parent = 0);
    void set_tree(MLRRTstar* p_tree);   
  signals:
  
  public slots:

  protected:

  private:


    MLRRTstar* mp_tree; 
  };
}

#endif /* MLRRTSTAR_VIZ_H_ */
