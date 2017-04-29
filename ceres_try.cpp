#include <iostream>
#include <string>
#include <fstream>

#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
using namespace std;


#include <ceres/ceres.h>
#include <Eigen/Dense>

// Class to represent Nodes
class Node
{
    public:
    /*Node()
    {

    }*/

    Node(int index, double x, double y, double theta)
    {
        this->index = index;
        this->x = x;
        this->y = y;
        this->theta = theta;
        p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = theta;
    }


    double x, y, theta;
    int index;
    double *p;
};


// Class to represent Edges
class Edge
{
public:
    Edge(const Node* a, const Node* b )
    {
        this->a = a;
        this->b = b;
    }

    void setEdgePose( double x, double y, double theta )
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    void setInformationMatrix( double I11, double  I12, double  I13, double I22, double I23, double I33 )
    {
        this->I11 = I11;
        this->I12 = I12;
        this->I13 = I13;
        this->I22 = I22;
        this->I23 = I23;
        this->I33 = I33;
    }

    const Node *a, *b;
    double x, y, theta;
    double I11, I12, I13, I22, I23, I33;

};


class ReadG2O
{
public:
    ReadG2O(const string& fName)
    {
      // Read the file in g2o format
        fstream fp;
        fp.open(fName.c_str(), ios::in);


        string line;
        int v = 0;
        int e = 0;
        while( std::getline(fp, line) )
        {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
            if( words[0].compare( "VERTEX_SE2") == 0 )
            {
                v++;
                int node_index = boost::lexical_cast<int>( words[1] );
                double x = boost::lexical_cast<double>( words[2] );
                double y = boost::lexical_cast<double>( words[3] );
                double theta = boost::lexical_cast<double>( words[4] );

                Node * node = new Node(node_index, x, y, theta);
                nNodes.push_back( node );
            }


            if( words[0].compare( "EDGE_SE2") == 0 )
            {
              // cout << e << words[0] << endl;
                int a_indx = boost::lexical_cast<int>( words[1] );
                int b_indx = boost::lexical_cast<int>( words[2] );

                double dx = boost::lexical_cast<double>( words[3] );
                double dy = boost::lexical_cast<double>( words[4] );
                double dtheta = boost::lexical_cast<double>( words[5] );

                double I11, I12, I13, I22, I23, I33;
                I11 = boost::lexical_cast<double>( words[6] );
                I12 = boost::lexical_cast<double>( words[7] );
                I13 = boost::lexical_cast<double>( words[8] );
                I22 = boost::lexical_cast<double>( words[9] );
                I23 = boost::lexical_cast<double>( words[10] );
                I33 = boost::lexical_cast<double>( words[11] );


                Edge * edge = new Edge( nNodes[a_indx], nNodes[b_indx] );
                edge->setEdgePose(dx, dy, dtheta);
                edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);


                nEdges.push_back(edge);
                e++;
            }

        }

    }


    // write nodes to file to be visualized with python script
    void writePoseGraph( const string& fname )
    {
      cout << "writePoseGraph : " << fname << endl;
      fstream fp;
      fp.open( fname.c_str(), ios::out );
      for( int i=0 ; i<this->nNodes.size() ; i++ )
      {
        fp << nNodes[i]->index << " " << nNodes[i]->p[0] << " " << nNodes[i]->p[1] << " " << nNodes[i]->p[2]  << endl;
      }
      // fp << "hello\n";
      // fp << "hello\n";


    }

//private:
    vector<Node*> nNodes; //storage for node
    vector<Edge*> nEdges; //storage for edges
};

struct PoseResidue
{
    // Observation for the edge
    PoseResidue(double dx, double dy, double dtheta)
    {

        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;

        // make a_Tcap_b
        double cos_t = cos( this->dtheta );
        double sin_t = sin( this->dtheta );
        a_Tcap_b(0,0) = cos_t;
        a_Tcap_b(0,1) = -sin_t;
        a_Tcap_b(1,0) = sin_t;
        a_Tcap_b(1,1) = cos_t;
        a_Tcap_b(0,2) = this->dx;
        a_Tcap_b(1,2) = this->dy;

        a_Tcap_b(2,0) = 0.0;
        a_Tcap_b(2,1) = 0.0;
        a_Tcap_b(2,2) = 1.0;

    }

    // Define the residue for each edge. P1 and P2 are 3-vectors representing state of the node ie. x,y,theta
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, T* e) const
    {

        // Convert P1 to T1 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_a;
        T cos_t = T(cos( P1[2] ));
        T sin_t = T(sin( P1[2] ));
        w_T_a(0,0) = cos_t;
        w_T_a(0,1) = -sin_t;
        w_T_a(1,0) = sin_t;
        w_T_a(1,1) = cos_t;
        w_T_a(0,2) = P1[0];
        w_T_a(1,2) = P1[1];

        w_T_a(2,0) = T(0.0);
        w_T_a(2,1) = T(0.0);
        w_T_a(2,2) = T(1.0);



        // Convert P2 to T2 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_b;
        cos_t = cos( P2[2] );
        sin_t = sin( P2[2] );
        w_T_b(0,0) = cos_t;
        w_T_b(0,1) = -sin_t;
        w_T_b(1,0) = sin_t;
        w_T_b(1,1) = cos_t;
        w_T_b(0,2) = P2[0];
        w_T_b(1,2) = P2[1];

        w_T_b(2,0) = T(0.0);
        w_T_b(2,1) = T(0.0);
        w_T_b(2,2) = T(1.0);

        Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
        T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                        T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                        T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));
        // now we have :: w_T_a, w_T_b and a_Tcap_b
        Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

        e[0] = diff(0,2);
        e[1] = diff(1,2);
        e[2] = asin( diff(1,0) );

        return true;
    }

    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double,3,3> a_Tcap_b;

    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta){
        return (new ceres::AutoDiffCostFunction<PoseResidue, 3, 3, 3>(
            new PoseResidue(dx, dy, dtheta)));
    };

};


int main()
{
    string fname = "../input_M3500_g2o.g2o";
    ReadG2O g( fname );

    // Write pose graph before optimization
    g.writePoseGraph("../init.txt");
    cout << "total nodes : "<< g.nNodes.size() << endl;
    cout << "total edges : "<< g.nEdges.size() << endl;


    ceres::Problem problem;
    for( int i=0 ; i<g.nEdges.size() ; i++ )
    {
      Edge* ed = g.nEdges[i];
      ceres::CostFunction * cost_function = PoseResidue::Create( ed->x, ed->y, ed->theta );
            // new ceres::AutoDiffCostFunction<PoseResidue, 3, 3, 3>( new PoseResidue(ed->x, ed->y, ed->theta) );

      problem.AddResidualBlock( cost_function, new ceres::HuberLoss(0.01), ed->a->p, ed->b->p );
      // cout << ed->a->index << "---> " << ed->b->index << endl;

      // Dry eval the function - optional
      double res[3];
      double *params[2];
      params[0] = ed->a->p;
      params[1] = ed->b->p;
      cost_function->Evaluate( params, res, NULL );
      cout << "Edge Cost : "<< res[0] << " " << res[1] << " " << res[2] << endl;
      // cout << res[2] << endl;

    }



    problem.SetParameterBlockConstant(g.nNodes[0]->p); //1st pose be origin

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // Write Pose Graph after Optimization
    g.writePoseGraph("../after_opt.txt");
}
