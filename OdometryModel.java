package exercises.e1;

import robot.Robot;

/**
 * Created by Theo Theodoridis.
 * Class    : OdometryModel
 * Version  : v1.0
 * Date     : © Copyright 2022
 * User     : ttheod
 * email    : t.theodoridis@salford.ac.uk
 * Comments : Implementation of an odometry model used to navigate a mobile robot in set of predefined coordinate
 *            waypoints.
 **/

public class OdometryModel
{
    private static final double GAMMA_TH = 2.5;
    private static final double GAMMA_D  = 50.0;

    private static final int TURN = 0;
    private static final int MOVE = 1;

    private int step = MOVE;
    private int i = 0;

    private static double NODE[][] =
    {
        {0,0},
         {2200, 0},
          {2200, -2200},
            {3900,-2450},
              {3900,-3750},
                {-1000,-3750}
    };

    private Robot robot;

   /**
    * Method     : OdometryModel::OdometryModel()
    * Purpose    : Secondary OdometryModel constructor.
    * Parameters : - robot : An object of the Robot class.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public OdometryModel(Robot robot)
    {
        this.robot = robot;
    }

   /**
    * Method     : OdometryModel::getAngle()
    * Purpose    : To get the tangent angle that points to a pair of coordinates.
    * Parameters : - nX : The node x coordinate.
    *              - nY : The node y coordinate.
    * Returns    : The angle to turn.
    * Notes      : None.
    **/
    public double getAngle(double nX, double nY)
    {
        return(get360(getRadToDeg(Math.atan2(nY - robot.kinematics.getY(), nX - robot.kinematics.getX()))));
    }

    public double getAngle(double nX, double nY, double rX, double rY)
    {
        return(get360(getRadToDeg(Math.atan2(nY - rY, nX - rX))));
    }

   /**
    * Method     : OdometryModel::getRadToDeg()
    * Purpose    : To transform radians to degrees.
    * Parameters : - th : The theta angle.
    * Returns    : The degrees.
    * Notes      : None.
    **/
    public double getRadToDeg(double th)
    {
        return(Math.toDegrees(th));
    }

   /**
    * Method     : OdometryModel::get360()
    * Purpose    : To normalise the robot's th into [0, 360].
    * Parameters : - th : The theta angle.
    * Returns    : The normalized angle.
    * Notes      : None.
    **/
    public double get360(double th)
    {
        return(th - 360.0 * Math.floor(th / 360.0));
    }

   /**
    * Method     : OdometryModel::isAngularDestination()
    * Purpose    : To check if the robot has reached the node's angular destination.
    * Parameters : - nTh : The theta angle of the node.
    * Returns    : True when the angle (th) is reached, False otherwise.
    * Notes      : A threshold ensures that the robot will not miss the angle due to the drift error.
    **/
    public boolean isAngularDestination(double nTh)
    {
        double rTh = get360(robot.arRobot.getTh());
        if((rTh >= (nTh - GAMMA_TH)) && (rTh <= (nTh + GAMMA_TH)))
        return(true);
        return(false);
    }

   /**
    * Method     : OdometryModel::isLinearDestination()
    * Purpose    : To check if the robot has reached its linear destination using the Euclidean distance.
    * Parameters : - nX : The x coordinate of the node.
    *              - nY : The y coordinate of the node.
    * Returns    : True if the destination (x, y) is reached, False otherwise.
    * Notes      : The linear threshold is added to the x, y so that to reach a node more precisely.
    **/
    public boolean isLinearDestination(double nX, double nY)
    {
        double d = Math.sqrt(Math.pow(nX - robot.kinematics.getX(), 2) + Math.pow(nY - robot.kinematics.getY(), 2));
        if(d <= GAMMA_D)
        return(true);
        return(false);
    }

   /**
    * Method     : OdometryModel::navigator()
    * Purpose    : To implement waypoint navigation method.
    * Parameters : - vel : The robot velocity.
    * Returns    : True if navigation is completed (last node), False otherwise.
    * Notes      : None.
    **/
    public boolean navigator(double vel)
    {
        // [1]Extract node coordinates:
        double x  = NODE[i][0];
        double y  = NODE[i][1];
        double th = getAngle(x, y);

        switch(step)
        {
            // [2]Turn and stop to the node's location:
            case TURN:
            {
                robot.control.turnSpot(vel/2);
                if(isAngularDestination(th))
                {
                    robot.control.stop();
                    step = MOVE;
                }
            }break;
            // [3]Move and stop to the node's location:
            case MOVE:
            {
                robot.control.move(vel);
                if(isLinearDestination(x, y))
                {
                    robot.control.stop();
                    step = TURN;
                    if(++i == NODE.length)
                    {
                        return(true);
                    }
                }
            }break;
        }
        return(false);
    }
}
