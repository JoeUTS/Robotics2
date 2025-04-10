#include "salseman_solver.h"

SalsemanSolver() {

}

void setContourList(std::map<int, std::shared_ptr<Contour>> contourList) {
  contourList_ = contourList;
}

std::vector<std::pair<int, bool>> getTravelOrder(void) {
  return travelOrder_;
}