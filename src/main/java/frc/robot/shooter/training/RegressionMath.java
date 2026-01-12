package frc.robot.shooter.training;

public class RegressionMath {

    public static double[] solveLeastSquares(double[][] X, double[] y) {
        int rows = X.length;
        int cols = X[0].length;

        // Compute X^T X
        double[][] XtX = new double[cols][cols];
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < cols; j++) {
                double sum = 0;
                for (int k = 0; k < rows; k++) {
                    sum += X[k][i] * X[k][j];
                }
                XtX[i][j] = sum;
            }
        }

        // Compute X^T y
        double[] Xty = new double[cols];
        for (int i = 0; i < cols; i++) {
            double sum = 0;
            for (int k = 0; k < rows; k++) {
                sum += X[k][i] * y[k];
            }
            Xty[i] = sum;
        }

        // Solve (X^T X) * coeffs = X^T y
        return gaussianElimination(XtX, Xty);
    }

    private static double[] gaussianElimination(double[][] A, double[] b) {
        int n = b.length;

        for (int i = 0; i < n; i++) {

            // Pivot
            double max = Math.abs(A[i][i]);
            int maxRow = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(A[k][i]) > max) {
                    max = Math.abs(A[k][i]);
                    maxRow = k;
                }
            }

            // Swap rows
            double[] temp = A[i];
            A[i] = A[maxRow];
            A[maxRow] = temp;

            double t = b[i];
            b[i] = b[maxRow];
            b[maxRow] = t;

            // Eliminate
            for (int k = i + 1; k < n; k++) {
                double factor = A[k][i] / A[i][i];
                b[k] -= factor * b[i];
                for (int j = i; j < n; j++) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }

        // Back substitution
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = b[i];
            for (int j = i + 1; j < n; j++) {
                sum -= A[i][j] * x[j];
            }
            x[i] = sum / A[i][i];
        }

        return x;
    }
}
