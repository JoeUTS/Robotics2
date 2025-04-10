#include "salesman_solver.h"

SalesmanSolver::SalesmanSolver() {

}

void SalesmanSolver::setContourList(std::map<int, std::shared_ptr<Contour>> contourList) {
  contourList_ = contourList;
}

std::vector<std::pair<int, bool>> SalesmanSolver::getTravelOrder(void) {
  return travelOrder_;
}