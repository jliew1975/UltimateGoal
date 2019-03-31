package com.disnodeteam.dogecv.detectors.roverruckus;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.listeners.DetectorListener;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.HOGDescriptor;

import java.util.ArrayList;
import java.util.List;

public class SamplingOrderDetectorExt extends DogeCVDetector {
    public DetectorListener listener = null;
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

    public RatioScorer ratioScorer = new RatioScorer(1.0,5);
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer(0.01);
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05);


    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW,100);
    public DogeCVColorFilter whiteFilter  = new HSVRangeFilter(new Scalar(0,0,200), new Scalar(50,40,255));

    private SamplingOrder currentOrder = SamplingOrder.UNKNOWN;
    private SamplingOrder lastOrder    = SamplingOrder.UNKNOWN;
    private Rect foundRect    = null;

    private boolean found   = false;
    private boolean aligned = false;
    private double goldXPos = 0;

    private Mat workingMat  = new Mat();
    private Mat yellowMask  = new Mat();
    private Mat whiteMask   = new Mat();
    private Mat hiarchy     = new Mat();

    private Size stretchKernal = new Size(10,10);
    private Size newSize = new Size();

    public double alignPosOffset = 0;
    public double alignSize = 100;

    public boolean disableSampling = false;

    public SamplingOrderDetectorExt() {
        super();
        this.detectorName = "Sampling Order Detector";
    }

    @Override
    public Mat process(Mat input) {
        if(input.channels() < 0 || input.cols() <= 0){
            Log.e("DogeCV", "Bad INPUT MAT!");
        }
        input.copyTo(workingMat);
        input.release();


        yellowFilter.process(workingMat.clone(),yellowMask);
        whiteFilter.process(workingMat.clone(), whiteMask);


        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursWhite = new ArrayList<>();

        Imgproc.blur(whiteMask,whiteMask,new Size(2,2));
        Imgproc.blur(yellowMask,yellowMask,new Size(2,2));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursYellow,-1,new Scalar(230,70,70),2);

        Imgproc.findContours(whiteMask, contoursWhite, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursWhite,-1,new Scalar(230,70,70),2);


        Rect   chosenYellowRect  = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursYellow){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double diffrenceScore = calculateScore(points);

            if(diffrenceScore < chosenYellowScore && diffrenceScore < maxDifference ){
                chosenYellowScore = diffrenceScore;
                chosenYellowRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if( area > 500){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }

        List<Rect>   choosenWhiteRect  = new ArrayList<>();
        List<Double> chosenWhiteScore  = new ArrayList<>();;

        for(MatOfPoint c : contoursWhite){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double diffrenceScore = calculateScore(points);

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if( area > 1000){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
                Imgproc.putText(workingMat,"Diff: " + diffrenceScore,new Point(centerPoint.x, centerPoint.y + 20),0,0.5,new Scalar(0,255,255));
            }

            boolean good = true;
            if(diffrenceScore < maxDifference && area > 1000){
                for(Rect checkRect : choosenWhiteRect){
                    boolean inX = ( rect.x > (checkRect.x - (checkRect.width / 2))) && rect.x < (checkRect.x + (checkRect.width / 2));
                    boolean inY = ( rect.y > (checkRect.y - (checkRect.height / 2))) && rect.y < (checkRect.y + (checkRect.height / 2));
                    if(inX && inY){
                        good = false;
                    }
                }
                if(good){
                    choosenWhiteRect.add(rect);
                    chosenWhiteScore.add(diffrenceScore);
                }
            }


        }

        double alignX = (getAdjustedSize().width / 2) + alignPosOffset;
        double alignXMin = alignX - (alignSize / 2);
        double alignXMax = alignX +(alignSize / 2);
        double xPos = 0;

        if(chosenYellowRect != null){
            Imgproc.rectangle(workingMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double)chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

            xPos = chosenYellowRect.x + (chosenYellowRect.width / 2);
            goldXPos = xPos;
            Imgproc.circle(workingMat, new Point( xPos, chosenYellowRect.y + (chosenYellowRect.height / 2)), 5, new Scalar(0,255,0),2);

            if(xPos < alignXMax && xPos > alignXMin) {
                aligned = true;
            } else {
                aligned = false;
            }

            Imgproc.line(workingMat,new Point(xPos, getAdjustedSize().height), new Point(xPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);

            if(choosenWhiteRect.size() < 2) {
                found = true;
            }
        }

        if(choosenWhiteRect != null) {
            for(int i=0;i<choosenWhiteRect.size();i++){
                Rect rect = choosenWhiteRect.get(i);
                double score = chosenWhiteScore.get(i);
                Imgproc.rectangle(workingMat,
                        new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height),
                        new Scalar(255, 255, 255), 2);
                Imgproc.putText(workingMat,
                        "Silver: " + String.format("Score %.2f ", score) ,
                        new Point(rect.x - 5, rect.y - 10),
                        Core.FONT_HERSHEY_PLAIN,
                        1.3,
                        new Scalar(255, 255, 255),
                        2);
            }
        }

        if(!disableSampling) {
            if (choosenWhiteRect.size() != 0 && chosenYellowRect != null) {
                int leftCount = 0;
                for (int i = 0; i < choosenWhiteRect.size(); i++) {
                    Rect rect = choosenWhiteRect.get(i);
                    if (chosenYellowRect.x > rect.x) {
                        leftCount++;
                    }
                }

                if (leftCount == 0) {
                    currentOrder = SamplingOrder.LEFT;
                }

                if (leftCount == 1) {
                    currentOrder = SamplingOrder.CENTER;
                }

                if (leftCount >= 2 && choosenWhiteRect.size() >= 2) {
                    currentOrder = SamplingOrder.RIGHT;
                }
                found = true;
                lastOrder = currentOrder;
            } else if (choosenWhiteRect.size() >= 2 && chosenYellowRect == null) {
                currentOrder = SamplingOrder.RIGHT;
                found = true;
                lastOrder = currentOrder;
            } else {
                currentOrder = SamplingOrder.UNKNOWN;

                if (chosenYellowRect == null) {
                    found = false;
                }
            }
        }

        Imgproc.line(workingMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        Imgproc.line(workingMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);

        Imgproc.putText(workingMat,"Gold Position: " + lastOrder.toString(),new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,0),1);
        Imgproc.putText(workingMat,"Is Aligned: " + aligned,new Point(10,getAdjustedSize().height - 30),0,0.5, new Scalar(255,255,0),1);

        if(!disableSampling && listener != null) {
            listener.onEvent();
        }

        return workingMat;
    }

    @Override
    public void useDefaults() {
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
        addScorer(ratioScorer);
    }

    public void disableSampling() {
        this.disableSampling = true;
    }

    @Override
    public void disable() {
        super.disable();

        workingMat.release();
        yellowMask.release();
        whiteMask.release();
        hiarchy.release();
    }

    public boolean isFound() {
        return found;
    }

    public SamplingOrder getCurrentOrder() {
        return currentOrder;
    }

    public SamplingOrder getLastOrder() {
        return lastOrder;
    }

    public boolean isAligned(){
        return aligned;
    }

    public double getXPosition(){
        return goldXPos;
    }
}
