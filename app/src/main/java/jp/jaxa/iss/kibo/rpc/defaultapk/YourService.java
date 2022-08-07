package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;

import gov.nasa.arc.astrobee.Robot;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() { //where the main program is
        api.startMission(); //start the mission
        double angle = Math.sqrt(2) / 2; //use to define the z of quaternion

        //P1 10.71f, -7.76f, 4.4f
        Point per1 = new Point(10.71000, -7.780000, 4.60000); //define  the point before p1, inorder to avoid touching the Keep out zone.
        Point P1 = new Point(10.71f, -7.76f, 4.4f);//define the point 1
        Quaternion Q1 = new Quaternion(0f, (float) angle, 0f, (float) angle);

        try{ //use try catch to prevent crash
            specificMoveTo(per1,Q1);// feedback control of moving
            api.moveTo(P1,Q1,false); // move to p1
        }catch (Exception e){
            api.moveTo(per1, Q1, false);
            api.moveTo(P1,Q1,false);
        }
        waiting();// make the robot stop for 300 ms
        api.reportPoint1Arrival();//report to system that we have reach point 1
        waiting();

        aimLaser("target1");//adjust the robot, make the laser aiming to the circle of target
        waiting();

        //shot and take picture
        api.laserControl(true);//open the laser
        api.takeTarget1Snapshot();//shot the laser
        takePicture("target_1");//take a picture for test
        api.laserControl(false);//close the laser
        waiting();


        //Point S1 = new Point(10.68068,-8.37976,5.29881);
        Point S1 = new Point(10.55068, -8.37976, 5.4325); // a broke point from p1 to p2, in order to stay away form Keep out zone
        Quaternion QS1 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S1, QS1, false);

        //move to S2
        Point S2 = new Point(10.55068, -9.8, 5.4325); // a broke point from p1 to p2, in order to stay away from Keep out zone
        Quaternion QS2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QS2, false);

        //move to point 2
        Point P2 = new Point(11.21360, -10, 5.4825); // move to point 2
        //11.17460,     ,5.29881
        Quaternion Q2 = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(P2,Q2, false);
        waiting();
        api.saveMatImage(api.getMatNavCam(), "before X = "+api.getRobotKinematics().getPosition().getX());// take a picture to get the data in simulator
        api.saveMatImage(api.getMatNavCam(), "before Y = "+api.getRobotKinematics().getPosition().getY());
        api.saveMatImage(api.getMatNavCam(), "before Z = "+api.getRobotKinematics().getPosition().getZ());



        try { // use try catch to prevent crash
            aim(); //aim with hough circle, and move to the center of the circle
            Mat img = api.getMatNavCam();
            api.saveMatImage(img, "aim success");
            waiting();
        }catch (Exception ignored){
            Mat correct = api.getMatNavCam();
            api.saveMatImage(correct, "aim crash");
        }

        waiting();


        //shot and take picture
        api.laserControl(true);
        api.takeTarget2Snapshot();
        takePicture("target_2");
        api.laserControl(false);

        //p2 - s2 - s1
        Quaternion QG = new Quaternion(0f, 0f, (float) -angle, (float) angle);
        api.moveTo(S2, QG, false);
        api.moveTo(S1,QG, false);

        //move to gaol position
        Point PG = new Point(11.27460, -7.89178, 4.96538);// move to the end game position
        api.moveTo(PG,QG,false);
        takePicture("goal");
        api.reportMissionCompletion();


    }

    public void takePicture(String tag) {
        Mat image = api.getMatNavCam();// save a picture from the front camera
        api.saveMatImage(image, tag); // save it to simulator
    }


    private void waiting() {
        try {
            Thread.sleep(300);// make the robot sleep for 300 ms
        } catch (Exception ignored) {
        }
    }


    private void aimLaser(String mode) { // use to adjust the quaternion, to ,ake the laser aiming to the center of circle


        switch (mode) {
            case "target1": // different position have different quaternion, needs to be separated
                Point pj1 = new Point(0.05, -0.1, api.getRobotKinematics().getPosition().getZ());
                Quaternion qj1 = api.getRobotKinematics().getOrientation();
                api.relativeMoveTo(pj1, qj1, false);
                break;
            case "target2":
                Point pj2 = new Point(api.getRobotKinematics().getPosition().getX()-0.1, api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()+0.05);
                Quaternion qj2 = api.getRobotKinematics().getOrientation();
                api.moveTo(pj2,qj2,false);
                break;
        }
    }

    private void specificMoveTo (Point p, Quaternion q) { //feedback control of the robot while moving
        double weighted = 2;//plus to the error
        double tolerance = 0.05d;// the error we can accept
        double error_pos;
        double error_posX, error_posY, error_posZ;
        int time2 = 0;
        Log.d("startfrom", api.getRobotKinematics().getPosition().toString());
        api.moveTo(p, q, false);//move to the point
        waiting();

        Quaternion Q = api.getRobotKinematics().getOrientation(); //get the information of robot now.
        do {
            double currentX = api.getRobotKinematics().getPosition().getX();
            double currentY = api.getRobotKinematics().getPosition().getY();
            double currentZ = api.getRobotKinematics().getPosition().getZ();
            error_pos = Math.abs(p.getX() - currentX) + Math.abs(p.getY() - //calculate the error
                    currentY) + Math.abs(p.getZ() - currentZ);
            error_posX = weighted*Math.abs(p.getX() - currentX);
            error_posY = weighted*Math.abs(p.getY() - currentY);
            error_posZ = weighted*Math.abs(p.getZ() - currentZ);
            api.relativeMoveTo(new Point(error_posX, error_posY, error_posZ), Q, false); //fix the error
            time2 ++;
        }while(error_pos > tolerance && time2 < 3); // limitation, we are not allow to use the loop repeatedly.
    }

    private void aim(){ //to get the center of the target by HoughCircles
        Mat img = api.getMatNavCam();// get the original picture
        api.saveMatImage(img, "aim start");
        Imgproc.medianBlur(img, img, 3); // blur the original picture
        api.saveMatImage(img, "blur success");
        Mat circles = new Mat();
        Imgproc.HoughCircles(img, circles, Imgproc.HOUGH_GRADIENT, 1,
                (double)img.rows()/16, 300, 30, 20, 50); //find the circle
        api.saveMatImage(img, "find circle success");
        ArrayList<Integer> radius = new ArrayList<>();
        ArrayList<Double> pixelX = new ArrayList<>();
        ArrayList<Double> pixelY = new ArrayList<>();
        for (int i=0; i < circles.cols(); i++){ //get the circle into data
            double[] vCircle = circles.get(0, i);
            pixelX.add(vCircle[0]);
            pixelY.add(vCircle[1]);
            radius.add((int) Math.round(vCircle[2]));
            int Radius = (int)Math.round(vCircle[2]);
            org.opencv.core.Point center = new org.opencv.core.Point(vCircle[0], vCircle[1]); //draw the circle to the picture
            Imgproc.circle(img, center, Radius, new Scalar(0, 255, 0), 3, 8, 0);
            api.saveMatImage(img, "circle" + i); //get the picture with circle added
        }

        api.saveMatImage(img, "get circle value success");
        int max_radius = radius.get(0); //get the radius
        int index = 0;
        for (int i=0; i < radius.size(); i++){
            if (max_radius < radius.get(i)) { //get the biggest error
                max_radius = radius.get(i);
                index = i;
            }
        }
        api.saveMatImage(img, "pixelX.get(index)"+ pixelX.get(index)); // save a picture to get information
        api.saveMatImage(img, "pixelY.get(index)"+ pixelY.get(index));
        api.saveMatImage(img, "max_rasius:"+max_radius);
        api.saveMatImage(img, "find biggest circle success");
        double proportion = (double)max_radius / 0.05 ;
        double errorX = (pixelX.get(index) - 640) / proportion; //change the bit to angle
        double errorY = (pixelY.get(index) - 480) / proportion;
        api.saveMatImage(img, "errorX"+errorX);
        api.saveMatImage(img, "errorY"+errorY);
        double x2 = api.getRobotKinematics().getPosition().getX(); //new point
        double y2 = errorX;
        double z2 = errorY;
        api.saveMatImage(img, "adjust position success");
        Point p2 = new Point(x2, y2, z2);
        Quaternion q2 = api.getRobotKinematics().getOrientation();
        api.relativeMoveTo(p2,q2,false);// move to new point
        waiting();
        //api.moveTo(p2, q2, false);
        //specificMoveTo(p2,q2);
        api.saveMatImage(api.getMatNavCam(), "after X = "+api.getRobotKinematics().getPosition().getX());
        api.saveMatImage(api.getMatNavCam(), "after Y = "+api.getRobotKinematics().getPosition().getY());
        api.saveMatImage(api.getMatNavCam(), "after Z = "+api.getRobotKinematics().getPosition().getX());
        api.saveMatImage(img, "aim end");
    }


    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

}

