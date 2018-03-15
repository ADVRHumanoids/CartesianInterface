#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>

int main(int argc, char **argv)
{
    
    using namespace OpenSoT::solvers;
    
    const int n = 5;
    const int m = 1;
    
    Eigen::MatrixXd H(n,n), A(m,n);
    Eigen::VectorXd lA(m), uA(m), l(n), u(n), g(n), xopt;
    
    H.setRandom(n,n);
    H = H*H.transpose();
    
    std::cout << "H:\n" << H << std::endl;
    
    g.setRandom(n);
    
    A << 1, 1, 1, 1, 1;
    lA << -1;
    uA << 1;
    
    l.setConstant(n, -1.0);
    u = -l;
    
    QPOasesBackEnd qpoases_solver(n,m);
    
    qpoases_solver.initProblem(H, g, A, lA, uA, l, u);
    qpoases_solver.solve();
    xopt = qpoases_solver.getSolution();
    
    std::cout << "qpOASES:\n";
    std::cout << "Xopt: \n" << xopt << std::endl;
    std::cout << "Ax: \n" << A*xopt << std::endl;
    
    
    OSQPBackEnd osqp_solver(n,m);
    
    osqp_solver.initProblem(H, g, A, lA, uA, l, u);
    osqp_solver.solve();
    xopt = osqp_solver.getSolution();
    
    std::cout << "OSQP:\n";
    std::cout << "Xopt: \n" << xopt << std::endl;
    std::cout << "Ax: \n" << A*xopt << std::endl;
    
    H.setRandom(n,n);
    H = H*H.transpose();
    
    g.array() += 1.0;
    
    qpoases_solver.updateTask(H, g);
    osqp_solver.updateTask(H, g);
    
    qpoases_solver.solve();
    osqp_solver.solve();
    
    std::cout << "Xopt (QPOASES): " << qpoases_solver.getSolution().transpose() << std::endl;
    std::cout << "Xopt (OSQP): " << osqp_solver.getSolution().transpose() << std::endl;
    
    
    A << 1, 2, 3, 4, 5;
    lA /= 2.0;
    uA /= 2.0;
    
    qpoases_solver.updateConstraints(A, lA, uA);
    osqp_solver.updateConstraints(A, lA, uA);
    qpoases_solver.solve();
    osqp_solver.solve();
    
    std::cout << "Xopt (QPOASES): " << qpoases_solver.getSolution().transpose() << std::endl;
    std::cout << "Xopt (OSQP): " << osqp_solver.getSolution().transpose() << std::endl;
    
    
    l /= 10.0;
    u /= 10.0;
    
    qpoases_solver.updateBounds(l, u);
    osqp_solver.updateBounds(l, u);
    qpoases_solver.solve();
    osqp_solver.solve();
    
    std::cout << "Xopt (QPOASES): " << qpoases_solver.getSolution().transpose() << std::endl;
    std::cout << "Xopt (OSQP): " << osqp_solver.getSolution().transpose() << std::endl;
    
}