package com.example.dhparameterfx;

import java.util.ArrayList;
import java.util.List;

public class IKSolver {

    private final ForwardKinematicsEngine fkEngine = new ForwardKinematicsEngine();
    private static final double STEP_SIZE = 1e-4;

    public boolean isTargetReachable(List<DHParameterModel> dhModels, double[] targetPos) {
        if (dhModels.isEmpty()) return false;

        double maxReach = 0;
        for (DHParameterModel m : dhModels) {
            maxReach += Math.abs(m.getA()) + Math.abs(m.getD());
        }

        List<DHParameter> dhParams = getCurrentDHParams(dhModels);
        List<Matrix4x4> transforms = fkEngine.computeCumulativeTransforms(dhParams);

        double[] basePos = transforms.isEmpty() ? new double[]{0, 0, 0} : transforms.get(0).getPosition();

        double dx = targetPos[0] - basePos[0];
        double dy = targetPos[1] - basePos[1];
        double dz = targetPos[2] - basePos[2];
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        return dist <= (maxReach * 1.05);
    }

    public boolean solve(List<DHParameterModel> dhModels, double[] targetPos, int maxIterations, double tolerance) {
        return solve(dhModels, targetPos, maxIterations, tolerance, null);
    }

    /**
     * @param isPrismatic per-joint type: true = driven by d (prismatic), false/absent-index/null
     *                     list = driven by theta (revolute). Same length/order as dhModels.
     */
    public boolean solve(List<DHParameterModel> dhModels, double[] targetPos, int maxIterations, double tolerance,
                         List<Boolean> isPrismatic) {
        double lambda = 0.2; // Slightly higher damping for Z-axis stability
        double bestError = Double.MAX_VALUE;
        double[] bestQ = new double[dhModels.size()];

        int numJoints = dhModels.size();
        double maxCartesianStep = 1.0; // Max distance the end-effector can attempt to move per iteration

        for (int iter = 0; iter < maxIterations; iter++) {
            List<DHParameter> dhParams = getCurrentDHParams(dhModels);
            List<Matrix4x4> transforms = fkEngine.computeCumulativeTransforms(dhParams);

            if (transforms.isEmpty()) return false;

            double[] currentPos = transforms.get(transforms.size() - 1).getPosition();

            double ex = targetPos[0] - currentPos[0];
            double ey = targetPos[1] - currentPos[1];
            double ez = targetPos[2] - currentPos[2];

            double errorDist = Math.sqrt(ex * ex + ey * ey + ez * ez);

            // Track the best configuration found so far
            if (errorDist < bestError) {
                bestError = errorDist;
                for (int i = 0; i < numJoints; i++) {
                    bestQ[i] = isPrismatic(isPrismatic, i) ? dhModels.get(i).getD() : dhModels.get(i).getTheta();
                }
            }

            if (errorDist < tolerance) {
                return true;
            }

            // --- CARTESIAN ERROR CLAMPING ---
            // If the target is far away, scale down the error vector.
            // This forces the solver to take a small, stable step toward the target
            // rather than mathematically exploding.
            if (errorDist > maxCartesianStep) {
                double scale = maxCartesianStep / errorDist;
                ex *= scale;
                ey *= scale;
                ez *= scale;
            }

            double[][] J = computePositionJacobian(dhModels, currentPos, isPrismatic);

            // Compute Damped Pseudoinverse J_damped = J^T * (J * J^T + lambda^2 * I)^-1
            double[][] A = new double[3][3];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    double sum = 0;
                    for (int k = 0; k < numJoints; k++) {
                        sum += J[r][k] * J[c][k];
                    }
                    if (r == c) sum += lambda * lambda;
                    A[r][c] = sum;
                }
            }

            double[][] Ainv = invert3x3(A);
            if (Ainv == null) break;

            double[] dampedErr = new double[3];
            dampedErr[0] = Ainv[0][0] * ex + Ainv[0][1] * ey + Ainv[0][2] * ez;
            dampedErr[1] = Ainv[1][0] * ex + Ainv[1][1] * ey + Ainv[1][2] * ez;
            dampedErr[2] = Ainv[2][0] * ex + Ainv[2][1] * ey + Ainv[2][2] * ez;

            // Apply update with strict clamping
            for (int j = 0; j < numJoints; j++) {
                DHParameterModel model = dhModels.get(j);

                double rawDelta = J[0][j] * dampedErr[0] + J[1][j] * dampedErr[1] + J[2][j] * dampedErr[2];

                if (isPrismatic(isPrismatic, j)) {
                    // rawDelta is already in linear units (no radians/degrees conversion needed,
                    // since the Jacobian column for a prismatic joint was built by perturbing d
                    // directly - see computePositionJacobian).
                    double dLinear = rawDelta;

                    // Per-iteration step cap, analogous to the 10 deg/iteration cap below - keeps
                    // the solver from taking a huge jump toward a singularity.
                    if (dLinear > 2.0) dLinear = 2.0;
                    if (dLinear < -2.0) dLinear = -2.0;

                    double newD = model.getD() + dLinear;

                    // DHParameterModel doesn't expose configured min/max bounds for d (only for
                    // theta), so this is a generous safety bound rather than a precision limit.
                    if (newD > 50.0) newD = 50.0;
                    if (newD < -50.0) newD = -50.0;

                    model.dProperty().set(newD);
                } else {
                    // Calculate required movement for this joint in degrees
                    double dqDeg = Math.toDegrees(rawDelta);

                    // Keep the angular step cap to prevent singularity teleportation
                    if (dqDeg > 10.0) dqDeg = 10.0;
                    if (dqDeg < -10.0) dqDeg = -10.0;

                    double newTheta = model.getTheta() + dqDeg;

                    // Hard clamp to configured joint bounds
                    if (newTheta > model.getMaxTheta()) newTheta = model.getMaxTheta();
                    if (newTheta < model.getMinTheta()) newTheta = model.getMinTheta();

                    model.setTheta(newTheta);
                }

                // CARTESIAN ERROR CLAMPING & FLOOR GUARD
                if (errorDist > maxCartesianStep) {

                    double scale = maxCartesianStep / errorDist;
                    ex *= scale;
                    ey *= scale;
                    ez *= scale;

                }

                // FLOOR CONSTRAINT: Prevent the solver from driving the target underground
                if (currentPos[2] + ez < 0.0) {

                    ez = -currentPos[2]; // Cap the downward movement exactly at the floor (Z=0)

                }
            }
        }

        // Restore the best configuration achieved
        for (int i = 0; i < numJoints; i++) {
            if (isPrismatic(isPrismatic, i)) {
                dhModels.get(i).dProperty().set(bestQ[i]);
            } else {
                dhModels.get(i).setTheta(bestQ[i]);
            }
        }

        return bestError <= 0.5;
    }

    private boolean isPrismatic(List<Boolean> isPrismatic, int jointIndex) {
        return isPrismatic != null && jointIndex < isPrismatic.size() && Boolean.TRUE.equals(isPrismatic.get(jointIndex));
    }

    private double[][] computePositionJacobian(List<DHParameterModel> dhModels, double[] currentPos, List<Boolean> isPrismatic) {
        int n = dhModels.size();
        double[][] J = new double[3][n];

        for (int j = 0; j < n; j++) {
            DHParameterModel model = dhModels.get(j);
            double[] posP;

            if (isPrismatic(isPrismatic, j)) {
                // Prismatic: perturb d directly. Already in the same linear units as position,
                // so no radians/degrees conversion is needed (unlike the theta case below).
                double origD = model.getD();
                model.dProperty().set(origD + STEP_SIZE);
                List<Matrix4x4> transformsP = fkEngine.computeCumulativeTransforms(getCurrentDHParams(dhModels));
                posP = transformsP.get(transformsP.size() - 1).getPosition();
                model.dProperty().set(origD);
            } else {
                double origTheta = model.getTheta();
                model.setTheta(origTheta + Math.toDegrees(STEP_SIZE));
                List<Matrix4x4> transformsP = fkEngine.computeCumulativeTransforms(getCurrentDHParams(dhModels));
                posP = transformsP.get(transformsP.size() - 1).getPosition();
                model.setTheta(origTheta);
            }

            J[0][j] = (posP[0] - currentPos[0]) / STEP_SIZE;
            J[1][j] = (posP[1] - currentPos[1]) / STEP_SIZE;
            J[2][j] = (posP[2] - currentPos[2]) / STEP_SIZE;
        }

        return J;
    }

    private List<DHParameter> getCurrentDHParams(List<DHParameterModel> dhModels) {
        List<DHParameter> list = new ArrayList<>();
        for (DHParameterModel m : dhModels) list.add(m.toDHParameter());
        return list;
    }

    private double[][] invert3x3(double[][] m) {
        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        if (Math.abs(det) < 1e-9) return null;

        double invdet = 1.0 / det;
        double[][] inv = new double[3][3];

        inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invdet;
        inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
        inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
        inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
        inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
        inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invdet;
        inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invdet;
        inv[2][1] = (m[0][1] * m[1][0] - m[0][0] * m[2][1]) * invdet;
        inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invdet;

        return inv;
    }
}