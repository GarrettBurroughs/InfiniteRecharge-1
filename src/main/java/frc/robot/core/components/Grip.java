package frc.robot.core.components;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.vision.VisionPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.*;


public class Grip implements VisionPipeline {
    private Mat rgbThresholdOutput = new Mat(); 
    private Mat resizeImageOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    public void process(Mat source0){
        //resize image 
        Mat resizeImageInput = source0; 
        double resizeImageWidth = 320.0; 
        double resizeImageHeight = 240.0; 
        int resizeImageInterpolation = Imgproc.INTER_CUBIC; 
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput); 
        
        //Take the resized image and edit the color output 
        Mat rgbThresholdInput = resizeImageOutput; 
        double[] rgbThresholdRed = {0,135};
        double[] rgbThresholdGreen = {148, 255};
        double[] rbgThresholdBlue = {0,255};
        
        rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rbgThresholdBlue, rgbThresholdOutput);
        //find contours output
        Mat findContoursInput = rgbThresholdOutput; 
        boolean findContoursExternalOnly = true;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput); 

        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput; 
        double filterContoursMinArea = 20.0;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 0.0;
        double filterContoursMaxWidth = 1000.0;
        double filterContoursMinHeight = 0.0;
        double filterContoursMaxHeight = 1000.0;
        double[] filterContoursSolidity = {0, 100};
        double filterContoursMaxVertices = 1000000.0;
        double filterContoursMinVertices = 0.0;
        double filterContoursMinRatio = 0.0;
        double filterContoursMaxRatio = 1000.0;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
    }

    private void resizeImage(Mat input, double width, double height, int interpolation, Mat output) {
            Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue, Mat out){
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
        Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
                new Scalar(red[1], green[1], blue[1]), out);
    }
    private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours){
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
            double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                    maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                    minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();

        for (int i = 0; i < inputContours.size(); i++) {
        final MatOfPoint contour = inputContours.get(i);
        final Rect bb = Imgproc.boundingRect(contour);
        if (bb.width < minWidth || bb.width > maxWidth) continue;
        if (bb.height < minHeight || bb.height > maxHeight) continue;
        final double area = Imgproc.contourArea(contour);
        if (area < minArea) continue;
        if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
        Imgproc.convexHull(contour, hull);
        MatOfPoint mopHull = new MatOfPoint();
        mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
        for (int j = 0; j < hull.size().height; j++) {
        int index = (int)hull.get(j, 0)[0];
        double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
        mopHull.put(j, 0, point);
        }
        final double solid = 100 * area / Imgproc.contourArea(mopHull);
        if (solid < solidity[0] || solid > solidity[1]) continue;
        if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
        final double ratio = bb.width / (double)bb.height;
        if (ratio < minRatio || ratio > maxRatio) continue;
        output.add(contour);
        }
    }   
}
