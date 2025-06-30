#include "robotKinematics.hpp"
#include "../Eigen/Dense"

#include <qpOASES.hpp>

namespace kinematics {

Joint::Joint() {
    transformationMatrix = Matrix4d::Identity();

    lowerLimit = 0.0;
    upperLimit = 0.0;

    type = JointType::Linear;
}

Joint::Joint(Matrix4d inputMatrix, JointType inputType, double inputNegLimit, double inputPosLimit) {
    transformationMatrix = inputMatrix;

    lowerLimit = inputNegLimit;
    upperLimit = inputPosLimit;

    type = inputType;
}


// public:
RobotKinematics::RobotKinematics() {
    transformationMatrix = Matrix4d::Identity();
}

std::vector<Matrix4d> RobotKinematics::forwardKinematics(VectorXd& jointStates) {
    if (jointStates.rows() != joints.size()) 
        throw std::runtime_error("Given joint state has an unexpected size!\n");

    std::vector<Matrix4d> results;
    results.emplace_back(transformationMatrix);

    size_t index = 0;
    for (const auto& joint : joints)
    {
        const double& state = jointStates(index);

        // check if joint state is within the defined limits
        if (state < joint.lowerLimit - 1e-6 || state > joint.upperLimit + 1e-6) 
            throw std::runtime_error("Given joint states contains an out of bounds element!\n");

        Matrix4d matrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            matrix.col(3) += matrix.col(2) * state;
        } 
        else if (joint.type == JointType::Rotation) {
            matrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(state, Eigen::Vector3d::UnitZ()).toRotationMatrix();;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1); i.e. the frame of joint i-1 expressed in the world frame
        // since we want World -> Joint_(i), we do:  {World -> Joint_(i-1)} * {Joint_(i-1) -> Joint_(i)}
        results.emplace_back(results.back() * matrix);

        ++index;
    }

    results.emplace_back(results.back() * endEffector.transformationMatrix);

    return results;
}

Matrix4d RobotKinematics::fastForwardKinematics(VectorXd& jointStates) {
    if (jointStates.rows() != joints.size()) 
        throw std::runtime_error("Given joint state has an unexpected size!\n");

    Matrix4d result = transformationMatrix;
    Matrix4d tempMatrix;
    const double tol = 1e-6;
    
    size_t index = 0;
    for (const auto& joint : joints)
    {
        const double& state = jointStates(index);

        // check if joint state is within the defined limits
        if (state < (joint.lowerLimit - tol) || state > (joint.upperLimit + tol)) 
            throw std::runtime_error("Joint state out of bounds!\n");

        tempMatrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            tempMatrix.col(3) += tempMatrix.col(2) * state;
        } 
        else if (joint.type == JointType::Rotation) {
            tempMatrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(state, Eigen::Vector3d::UnitZ()).toRotationMatrix();;
        }
        
        // results.back() refers to the matrix going from World -> Joint_(i-1); i.e. the frame of joint i-1 expressed in the world frame
        // since we want World -> Joint_(i), we do:  {World -> Joint_(i-1)} * {Joint_(i-1) -> Joint_(i)}
        result *= tempMatrix;
        ++index;
    }

    result *= endEffector.transformationMatrix;
    return result;
}

VectorXd RobotKinematics::inverseKinematics(Matrix4d& goal, int maxIterations, double tolerance) {
    /* This algorithm attemps to solve (redundant) inverse kinematics by solving the following problem:
        $\text{min}_x \;f(\textbf{x}) $
        $\text{subject to} \; h_i(\textbf{x}) \ge 0, \; i \in (0,1,2...m)$
        where $\textbf{x}$ is a vector containing the joint states.
        A basic SQP approach is used here. This approach works by linearizing the problem into:
        $\text{min}_p \; f(\textbf{x}_k) + (\grad f(\textbf{x}_k))^T p + \frac{1}{2}p^T\grad_{xx}^2\mathcal{L}_k p$
        $\text{subject to} \; (\grad h_i(\textbf{x}_k))^T p + h_i(\textbf{x}_k) \ge 0, \; i \in (0,1,2...m)$
        where $\textbf{x}_k$ is the $k$'th guess of the optimal joint state, and $\mathcal{L}_k = \mathcal{L}(\textbf{x}_k, \textbf{l}_k)= f(\textbf{x}_k) - \textbf{l}_k^T\textbf{h}(\textbf{x}_k)$
        this new problem is solved using a tradition quadratic solver, which gives us an optimal $p$ and $l$, 
        with which we update the values for the next iteration: $\textbf{x}_{k+1}=\textbf{x}_k + \text{lineSearch}(p)$ and $\textbf{l}_{k+1}=\textbf{l}_k$
    */
    
    int n = joints.size();                  // n: number of joints
    int m = n * 2;                          // m: number of inequality constraints (for now we use 2 for each joint; 1 for the lower joint limit, 1 for the upper limit)

    VectorXd x = VectorXd::Zero(n);         // x: the current joint state guess. initial joints guess is set as a zero vector
    VectorXd l = VectorXd::Zero(m);         // l: the lagrange multiplier for inequalities. Initially set as a zero vector
    MatrixXd H = MatrixXd::Identity(n, n);  // H: approximation of the hessian  using BFGS 

    for (int k = 0; k < maxIterations; k++) {
        // calculate cost $f(\textbf{x}_k)$ algebraically
        Matrix4d effector = fastForwardKinematics(x);
        double cost = costFunction(effector, goal);

        // estimate $\grad f(\textbf{x}_k)$ using a central difference estimate for each element
        VectorXd costGradient(n);
        const double gradStepSize = 1e-6;
        for (int i = 0; i < n; i++) {
            VectorXd xPlus = x, xMin = x;
            xPlus(i) += gradStepSize;
            xMin(i) -= gradStepSize;

            Matrix4d effectorPlus = fastForwardKinematics(xPlus);
            Matrix4d effectorMin = fastForwardKinematics(xMin);
            costGradient(i) = (costFunction(effectorPlus, goal) - costFunction(effectorMin, goal)) / (2.0 * gradStepSize);
        }

        // calculate $h_i(\textbf{x}_k)$ algebraically
        VectorXd constraints = jointConstraints(x);

        // calculate $\grad h_i(\textbf{x}_k)$ algebraically
        std::vector<VectorXd> constraintsGradients = jointConstraintsGradients(x);

        // solve the IQP problem (using OSQP or qpOASES?)

        // perform a line search for the optimal step size in the direction found by the IQP

        // set $\textbf{x}_{k+1}$ and $\textbf{l}_{k+1}$

        // check, using the KKT conditions, if our tolerance has been achieved

        // update $\grad_{xx}^2\mathcal{L}_k$, (H) approximation

    }

    return VectorXd::Zero(n);
}

// private:
double RobotKinematics::costFunction(Matrix4d& input, Matrix4d& goal) {
        //TODO: actually take into account the rotations
        return (input.topRightCorner<3,1>() - goal.topRightCorner<3,1>()).squaredNorm();
}

VectorXd RobotKinematics::jointConstraints(VectorXd& jointStates) {
    VectorXd result = VectorXd::Zero(joints.size() * 2);

    int index = 0;
    for (const auto& joint : joints) {

        // the constraint $b_{lower} \le x_i \le b_{upper}$ can be rewritten into $x_i - b_{lower} \ge 0$ and $b_{upper} - x_i \ge 0$
        result(2 * index + 0) = jointStates(index) - joint.lowerLimit;
        result(2 * index + 1) = joint.upperLimit - jointStates(index);

        ++index;
    }

    return result;
}

std::vector<VectorXd> RobotKinematics::jointConstraintsGradients(VectorXd& jointStates) {
    int numberOfJoints = joints.size();

    std::vector<VectorXd> results;
    results.reserve(numberOfJoints * 2);

    for (int i = 0; i < numberOfJoints; i++) {
        // the gradient for the constraints for joint limits are simply +1.0 and -1.0, depending on the corresponding bound being lower or upper
        results.emplace_back(VectorXd::Zero(numberOfJoints));
        results.back()(i) = 1.0;
        results.emplace_back(VectorXd::Zero(numberOfJoints));
        results.back()(i) = -1.0;
    }

    return results;
}

}