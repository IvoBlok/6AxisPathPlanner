#include "robotKinematics.hpp"
#include "CustomEigen.hpp"

#include "proxsuite/proxqp/dense/dense.hpp"
#include "proxsuite/proxqp/parallel/qp_solve.hpp"
#include <iostream>
#include <random>

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
    const double tol = 2e-3;

    size_t index = 0;
    for (const auto& joint : joints)
    {
        const double& state = jointStates(index);

        // check if joint state is within the defined limits
        if (state < (joint.lowerLimit - tol) || state > (joint.upperLimit + tol)) 
            throw std::runtime_error("Given joint states contains an out of bounds element!\n");

        Matrix4d matrix = joint.transformationMatrix;

        // apply the joint angle/translation to i
        if (joint.type == JointType::Linear) {
            matrix.col(3) += matrix.col(2) * state;
        } 
        else if (joint.type == JointType::Rotation) {
            matrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(state, Vector3d::UnitZ()).toRotationMatrix();;
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
    const double tol = 2e-3;
    
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
            tempMatrix.topLeftCorner<3,3>() *= Eigen::AngleAxisd(state, Vector3d::UnitZ()).toRotationMatrix();;
        }
        
        result *= tempMatrix;
        ++index;
    }

    result *= endEffector.transformationMatrix;
    return result;
}

IKResult RobotKinematics::inverseKinematics(Matrix4d& goal, const bool useRotation, Vector3d rotationAxisIgnore, int maxIterations, int maxAttempts, double tolerance, bool startAtLast) {
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
    
    IKResult result;

    const int n = joints.size();                    // n: number of joints
    const int m = n * 2;                            // n: number of joints
    VectorXd x(n);                                  // x: the current joint state guess

    proxsuite::proxqp::dense::QP<double> qp(n, 0, m); // n vars, 0 equality constraints, m inequality constraints
    VectorXd upperBounds = VectorXd::Constant(m, std::numeric_limits<double>::infinity());
    VectorXd lowerBounds;

    double cost;
    VectorXd costGradient;
    VectorXd constraints;
    MatrixXd constraintsGradients;
    MatrixXd H;
    
    VectorXd l;
    VectorXd p;

    VectorXd stationarity;
    VectorXd complSlack;

    VectorXd nextCostGradient;
    MatrixXd effector;
    
    VectorXd y;
    VectorXd s;
    double ys;
    VectorXd Hs;
    double sTHs;

    double theta;
    const double dampingThreshold = 0.2;

    int attempt;
    int k;
    for (attempt = 0; attempt < maxAttempts; attempt++) {
        if (attempt == 0 && startAtLast && lastIKResult.rows() != 0)
            x = lastIKResult;
        else
            x = randomJointStateInBounds();             // initial joints guess is a random state within the joint bounds
        
        H = MatrixXd::Identity(n, n);                   // H: approximation of the hessian  using BFGS 
        l = VectorXd::Zero(m);
        nextCostGradient = costGradientEstimate(x, goal, useRotation, rotationAxisIgnore, 1e-3);

        for (k = 0; k < maxIterations; k++) {
            // ------------- Calculate Problem Terms --------------------
            effector = fastForwardKinematics(x);
            cost = costFunction(effector, goal, useRotation, rotationAxisIgnore);

            // estimate $\grad f(\textbf{x}_k)$ using a central difference estimate for each element
            costGradient = nextCostGradient;

            // calculate $h_i(\textbf{x}_k)$ algebraically
            constraints = jointConstraints(x);

            // calculate $\grad h_i(\textbf{x}_k)$ algebraically
            constraintsGradients = jointConstraintsGradients(x); // (n x m), each column being the gradient of a constraint
            constraintsGradients.transposeInPlace();   // (m x n)
            
            // Nudge the hessian a bit, and ensure it is symmetric
            for(int i=0; i<n; i++) H(i,i) += 1e-6;
            H = 0.5 * (H + H.transpose());

            lowerBounds = -constraints;

            // ------------- Check Convergence --------------------------
            stationarity = costGradient + constraintsGradients.transpose() * l;
            complSlack = l.cwiseProduct(constraints);

            // check all 4 separate conditions of KKT, and the absolute cost:
            if ((stationarity.norm() <= tolerance &&                // Stationarity: $||\grad f_k + \sum_{i} l_{i,k} \grad h_{i,k}|| \le \text{tolerance}$
                (constraints.array() >= -tolerance).all() &&        // Primal feasability: $h_{i,k}\ge -\text{tolerance}$
                (l.array() >= -tolerance).all() &&                  // Dual feasability: $l_{i,k} \ge -\text{tolerance}$
                complSlack.cwiseAbs().maxCoeff() <= tolerance)      // Complementary slackness: $|l_{i,k}h_{i,k}| \le \text{tolerance}$
                || cost < tolerance) {                              // Optimal cost achieved: $f_k \le \text{tolerance}$
                
                break;
            }

            // ------------- Solve IQP Problem --------------------------
            if (k == 0)
                qp.init(
                    H,
                    costGradient,
                    proxsuite::nullopt,
                    proxsuite::nullopt,
                    constraintsGradients,
                    lowerBounds,
                    upperBounds
                );
            else 
                qp.update(
                    H,
                    costGradient,
                    proxsuite::nullopt,
                    proxsuite::nullopt,
                    proxsuite::nullopt,
                    lowerBounds,
                    proxsuite::nullopt
                );

            qp.solve();
            p = qp.results.x;         

            // ------------- Line Search --------------------------------
            // since for simplification the problem was linearized for each IQP call, the solution isn't necessarily a solution to the non-linearized problem. 
            // Hence we nudge around the linearized solution so that the non-linearized constraints tell us it is a valid solution.
            double alpha = 1.0;
            while (alpha > 1e-4) {
                VectorXd x_new = x + alpha * p;
                if (isValidState(x_new)) break;
                alpha *= 0.75;
            }

            // ------------- Update Variables ---------------------------
            // set $\textbf{x}_{k+1}$
            x += alpha * p;
            l = qp.results.z; 

            // ------------- Update Hessian -----------------------------
            // update $\grad_{xx}^2\mathcal{L}_k$, (H) approximation
            //
            // $\mathcal{L}_k := \nabla^2_{\textbf{xx}}f(\textbf{x}_k)-\sum_{i}l_{k,i}\nabla^2_{\textbf{xx}}h_i(\textbf{x}_k)$
            //
            // since $f(\textbf{x})$ is a numerical thingy, and hence we can't define an algebraic hessian for it, we need to use some approximation method for it
            // For this I use BFGS here, though there definitely is some better variant.
            // The second component, dependent on $h_i(\textbf{x})$, is algebraically defined (currently). 
            // Even better, for the simple constraints here, where we have constraints of the form $h_i(\textbf{x})=\pm x_{j} \pm b$, we have $\nabla^2_{\textbf{xx}}h_i(\textbf{x}_k)=0$
            // since the contribution from the constraints is 0, we can update H directly using the BFGS update rule.
            //
            // BFGS Hessian approximation update goes like: $H_{k+1}=H_k - \frac{H_k s_ks_k^TH_k^T}{s_k^tH_ks_k}+\frac{y_ky_k^T}{y_k^Ts_k}$
            // where $\textbf{s}_k = \textbf{x}_{k+1} - \textbf{x}_k$ and $\textbf{y}_k=\nabla \textbf{f}_{k+1} - \nabla \textbf{f}_k$
            
            nextCostGradient = costGradientEstimate(x, goal, useRotation, rotationAxisIgnore, 1e-3);
            y = nextCostGradient - costGradient;
            s = alpha * p;
            
            Hs = H * s;
            sTHs = s.dot(Hs);

            ys = y.dot(s);
            if (ys < dampingThreshold * sTHs) {
                theta = (0.8 * sTHs) / (sTHs - ys); // Powell
                y = theta * y + (1.0 - theta) * Hs;
            }
            
            // BFGS with modified y
            H -= (Hs * Hs.transpose()) / sTHs;
            H += (y * y.transpose()) / y.dot(s);
        }

        if (cost < tolerance) {
            lastIKResult = x;
            result.state = x;
            result.type = IKResultType::Success;
            return result;
        }
    }
    result.type = IKResultType::OutOfReach;
    return result;
}

PathValidationResult RobotKinematics::validatePath(const core::Polyline2_5D& polyline, int maxIterations, int maxAttempts, double tolerance, bool startAtLast) {
    // TODO validate the full path between the vertices; I'm currently only checking the vertex points, not the arcs/lines that join them
    const std::vector<core::PlineVertex2_5D>& vertices = polyline.vertexes();

    PathValidationResult validationResult;
    validationResult.type = IKResultType::Success;
    validationResult.states.reserve(vertices.size());

    Matrix4d goal;
    IKResult result;
    for (const auto& vertex : vertices) {
        goal = createTransformationMatrix(vertex.point, vertex.plane.normal);
        result = inverseKinematics(goal, true, vertex.plane.normal, maxIterations, maxAttempts, tolerance, startAtLast);

        if (result.type == IKResultType::Success) {
            validationResult.states.emplace_back(result.state);
        } else {
            validationResult.type = result.type;
            validationResult.states.clear();
            break;
        }
    }

    if (result.type == IKResultType::Success && polyline.isClosed()) {
        validationResult.states.emplace_back(validationResult.states.front());
    }

    return validationResult;
}

// private:
double RobotKinematics::costFunction(const Matrix4d& input, const Matrix4d& goal, const bool useRotation, Vector3d rotationAxisIgnore) {
    // position error cost contribution
    double totalCost = (input.topRightCorner<3,1>() - goal.topRightCorner<3,1>()).squaredNorm();

    // rotation error cost contribution
    if (useRotation) {
        Quaterniond qInput(input.block<3,3>(0,0)), qGoal(goal.block<3,3>(0,0));
        qInput.normalize(); 
        qGoal.normalize();
        Quaterniond qError = qGoal.conjugate() * qInput;
        Vector3d rotationError = 2.0 * qError.vec();

        if(rotationAxisIgnore.norm() > 1e-6) {
            rotationAxisIgnore.normalize();
            // project the total rotation error onto the direction we don't care about to get its contribution, and remove it
            rotationError -= rotationError.dot(rotationAxisIgnore) * rotationAxisIgnore;
        }

        double rotationCost = (rotationError).squaredNorm();
        
        // the rotation cost is weighted relative to the positionCost
        totalCost += 0.7 * rotationCost;
    }

    return totalCost;
}

VectorXd RobotKinematics::costGradientEstimate(const VectorXd& jointStates, const Matrix4d& goal, const bool useRotation, Vector3d rotationAxisIgnore, const double stepSize) {
    int n = joints.size();
    VectorXd result(n);

    for (int i = 0; i < n; i++) {
        VectorXd xPlus = jointStates, xMin = jointStates;
        xPlus(i) += stepSize;
        xMin(i) -= stepSize;

        Matrix4d effectorPlus = fastForwardKinematics(xPlus);
        Matrix4d effectorMin = fastForwardKinematics(xMin);

        double costPlus = costFunction(effectorPlus, goal, useRotation, rotationAxisIgnore);
        double costMin = costFunction(effectorMin, goal, useRotation, rotationAxisIgnore);

        result(i) = (costPlus - costMin) / (2.0 * stepSize);
    }

    return result;
}

VectorXd RobotKinematics::jointConstraints(const VectorXd& jointStates) {
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

MatrixXd RobotKinematics::jointConstraintsGradients(const VectorXd& jointStates) {
    int numberOfJoints = joints.size();

    MatrixXd results = MatrixXd::Zero(numberOfJoints, numberOfJoints * 2);

    for (int i = 0; i < numberOfJoints; i++) {
        // the gradient for the constraints for joint limits are simply +1.0 and -1.0, depending on the corresponding bound being lower or upper
        results(i, 2*i) = 1.0;
        results(i, 2*i + 1) = -1.0;
    }

    return results;
}

VectorXd RobotKinematics::jointLowerBounds() {
    VectorXd result(joints.size());

    int index = 0;
    for (const auto& joint : joints) {
        result[index] = joint.lowerLimit; 
        index++;
    }

    return result;
}

VectorXd RobotKinematics::jointUpperBounds() {
    VectorXd result(joints.size());

    int index = 0;
    for (const auto& joint : joints) {
        result[index] = joint.upperLimit; 
        index++;
    }

    return result;
}

bool RobotKinematics::isValidState(const VectorXd& jointStates) {
    int index = 0;
    for (const auto& joint : joints) {
        if (jointStates(index) < joint.lowerLimit || jointStates(index) > joint.upperLimit)
            return false;

        ++index;
    }
    return true;
}

VectorXd RobotKinematics::randomJointStateInBounds() {
    VectorXd result(joints.size());
    std::mt19937 gen{std::random_device{}()};

    int index = 0;
    for (const auto& joint : joints) {
        result[index] = std::uniform_real_distribution<double>{joint.lowerLimit, joint.upperLimit}(gen);

        index++;
    }

    return result;
}

}