// STL
#include <iostream>
#include <string>
using namespace std;

// g2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

int main(int argc ,char* argv[] )
{
    int maxIterations = 50;
    string outputfilename = "../data/sphere_after.g2o";
    string inputfilename  = "../data/sphere_bignoise_vertex3.g2o";

    auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();

    auto blockSover = g2o::make_unique<BlockSolverX>(std::move(linearSolver));

    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(std::move(blockSover));

    SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    ifstream ifs(inputfilename.c_str() );
    if( !ifs )
    {
        cerr<<"Unable to open file."<<endl;
        return 1;
    }

    optimizer.load(ifs);
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);

    optimizer.save(outputfilename.c_str() );

    return 0;




}
