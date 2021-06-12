import sympy as sym
from sympy import MatMul, Matrix, simplify, solve
from sympy import cos, sin, tan, atan2, acos
from sympy.utilities.lambdify import lambdify
from math import pi


class robot:
    def __init__(self):
        """Class holding the essential parameters for kinematics and homogenous transformation matrix of the robot model
        """

        self.theta_1 = sym.Symbol('theta1')
        self.theta_2 = sym.Symbol('theta2')
        self.x, self.y, self.z = sym.Symbol(
            'x'), sym.Symbol('y'), sym.Symbol('z')

        self.xEE, self.yEE, self.zEE = sym.Symbol(
            'xEE'), sym.Symbol('yEE'), sym.Symbol('zEE')

        self.t1 = simplify(Matrix([[cos(-pi/2), -sin(-pi/2)*cos(pi/2), sin(-pi/2)*sin(pi/2), 0.05*cos(0)],
                                   [sin(-pi/2), cos(-pi/2)*cos(pi/2), -
                                    cos(-pi/2)*sin(pi/2), 0.05*sin(0)],
                                   [0, sin(pi/2), cos(pi/2), 2],
                                   [0, 0, 0, 1]]))

        self.t2 = simplify(Matrix([[cos(self.theta_1), -sin(self.theta_1)*cos(0), sin(self.theta_1)*sin(0), 1*cos(self.theta_1)],
                                   [sin(self.theta_1), cos(self.theta_1)*cos(0), -
                                    cos(self.theta_1)*sin(0), 1*sin(self.theta_1)],
                                   [0, sin(0), cos(0), 0.1],
                                   [0, 0, 0, 1]]))

        self.t3 = Matrix([[cos(self.theta_2), -sin(self.theta_2)*cos(0), sin(self.theta_2)*sin(0), 1*cos(self.theta_2)],
                          [sin(self.theta_2), cos(self.theta_2)*cos(0), -
                           cos(self.theta_2)*sin(0), 1*sin(self.theta_2)],
                          [0, sin(0), cos(0), 0.1],
                          [0, 0, 0, 1]])
        self.print = False  # Change to print the transformation matrix
        self.getTMatrix()
        self.getForwardKinematics()
        self.getInverseKinematics()
        self.print = False

    def getTMatrix(self):
        """Det the transformation matrix of the robto

        Returns:
            T (sympy.symbolFunction): The sympy based symbolic function for calculating the transformation provided the theta.
        """
        self.T = MatMul(self.t1*self.t2*self.t3)
        self.T = simplify(self.T)

        if (self.print):
            print("\v")
            print("TRANSFORMATION MATRIX FOR ROBOT MODEL FROM BASE FRAME TO END EFFECTOR")
            print(self.T)

        return self.T

    def getForwardKinematics(self):
        """Get the function equation for the forward kinematics. Only the position of the model is considered and is appropriate

        Returns:
            px_function, py_function, pz_function(sympy lambdify function): The functions of the position matrix of the robot end effector
        """
        self.px = self.T[0, 3]
        self.py = self.T[1, 3]
        self.pz = self.T[2, 3]

        self.px_function = lambdify([self.theta_1, self.theta_2], self.px)
        self.py_function = lambdify([self.theta_1, self.theta_2], self.py)
        self.pz_function = lambdify([self.theta_1, self.theta_2], self.pz)

        # Correction
        self.theta1_modified = self.theta_1 + pi/2  # Due to correction in robot frame
        self.px_function = lambdify([self.theta_1, self.theta_2],
                                    cos(self.theta1_modified) +
                                    cos(self.theta1_modified+self.theta_2))

        self.pz_function = lambdify([self.theta_1, self.theta_2],
                                    sin(self.theta1_modified) + sin(self.theta1_modified + self.theta_2)+2)

        if (self.print):
            print("\v")
            print("FORWARD KINEMTICS OF THE MODEL IS")
            print("X: {}".format(self.px))
            print("Y: {}".format(self.py))
            print("Z: {}".format(self.pz))

        return self.px_function, self.py_function, self.pz_function

    def getInverseKinematics(self):
        """Get the inverse kinematics of the robot model

        Returns:
            theta1_function, theta2_function (sympy lambdify function): The theta functions given the position of the end-effector
        """
        self.x_modified = self.x
        self.z_modified = self.z - 2
        self.theta2 = [acos((self.x_modified*self.x_modified + self.z_modified*self.z_modified - 1 - 1)/2),
                       -acos((self.x_modified * self.x_modified + self.z_modified * self.z_modified - 1 - 1)/2)]
        self.theta1 = [atan2(self.z_modified, self.x_modified) - atan2(sin(self.theta_2), 1+cos(self.theta_2)),
                       atan2(self.z_modified, self.x_modified) + atan2(sin(self.theta_2), 1+cos(self.theta_2))]

        self.theta1_function = lambdify(
            [self.x, self.z, self.theta_2], self.theta1)
        self.theta2_function = lambdify(
            [self.x, self.z], self.theta2)
        return self.theta1_function, self.theta2_function


if __name__ == "__main__":
    dh = robot()
    theta1, theta2 = dh.getInverseKinematics()
    px, py, pz = dh.getForwardKinematics()

    print(px(0.5, 1), pz(0.5, 1))

    print(theta2(1, 2))
    print(theta1(1, 2, theta2(1, 2)))
