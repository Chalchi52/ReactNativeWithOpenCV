package com.reactlibrary;

import com.facebook.react.bridge.Promise;
import com.facebook.react.bridge.ReactApplicationContext;
import com.facebook.react.bridge.ReactContextBaseJavaModule;
import com.facebook.react.bridge.ReactMethod;
import com.facebook.react.bridge.Callback;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

import org.opencv.android.Utils;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import android.net.Uri;
import android.os.Environment;
import android.util.Base64;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class RNOpenCvLibraryModule extends ReactContextBaseJavaModule {

    private final ReactApplicationContext reactContext;
    private int witdh;
    private String rotateAngle;

    public RNOpenCvLibraryModule(ReactApplicationContext reactContext) {
        super(reactContext);
        this.reactContext = reactContext;
    }

    @Override
    public String getName() {
        return "RNOpenCvLibrary";
    }

    @ReactMethod
    public void checkForBlurryImage(String imageAsBase64, Callback errorCallback, Callback successCallback) {
        try {
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inDither = true;
            options.inPreferredConfig = Bitmap.Config.ARGB_8888;

            byte[] decodedString = Base64.decode(imageAsBase64, Base64.DEFAULT);
            Bitmap image = BitmapFactory.decodeByteArray(decodedString, 0, decodedString.length);


//      Bitmap image = decodeSampledBitmapFromFile(imageurl, 2000, 2000);
            int l = CvType.CV_8UC1; //8-bit grey scale image
            Mat matImage = new Mat();
            Utils.bitmapToMat(image, matImage);
            Mat matImageGrey = new Mat();
            Imgproc.cvtColor(matImage, matImageGrey, Imgproc.COLOR_BGR2GRAY);

            Bitmap destImage;
            destImage = Bitmap.createBitmap(image);
            Mat dst2 = new Mat();
            Utils.bitmapToMat(destImage, dst2);
            Mat laplacianImage = new Mat();
            dst2.convertTo(laplacianImage, l);
            Imgproc.Laplacian(matImageGrey, laplacianImage, CvType.CV_8U);
            Mat laplacianImage8bit = new Mat();
            laplacianImage.convertTo(laplacianImage8bit, l);

            Bitmap bmp = Bitmap.createBitmap(laplacianImage8bit.cols(), laplacianImage8bit.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(laplacianImage8bit, bmp);
            int[] pixels = new int[bmp.getHeight() * bmp.getWidth()];
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());
            int maxLap = -16777216; // 16m
            for (int pixel : pixels) {
                if (pixel > maxLap)
                    maxLap = pixel;
            }

//            int soglia = -6118750;
            int soglia = -8118750;
            if (maxLap <= soglia) {
                System.out.println("is blur image");
            }

            successCallback.invoke(maxLap <= soglia);
        } catch (Exception e) {
            errorCallback.invoke(e.getMessage());
        }
    }

    @ReactMethod
    public void rotateImage(String imagePath, Promise promise) {
        try {
            String path = Uri.parse(imagePath).getEncodedPath();
            Mat dataSrc = Imgcodecs.imread(path, Imgcodecs.IMREAD_COLOR);
            Bitmap imageSrc = BitmapFactory.decodeFile(path);
            Mat dataRot = new Mat();
            Mat dataSrc2 = new Mat();
            Imgproc.cvtColor(dataSrc, dataSrc2, Imgproc.COLOR_BGR2RGB);
            if (imageSrc == null)
                imageSrc = Bitmap.createBitmap(dataSrc.cols(), dataSrc.rows(), Bitmap.Config.ARGB_8888);
            Core.rotate(dataSrc2, dataRot, Core.ROTATE_90_CLOCKWISE);
            Bitmap imageOut = Bitmap.createBitmap(dataRot.cols(), dataRot.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(dataRot, imageOut, true);
            String[] tmp = path.split("/");
            String name = tmp[tmp.length - 1].replace(".jpg", "");
            saveFile(name, "_Original", imageSrc);
            saveFile(name, "_Rotate", imageOut);
            Mat rotated = rotate(dataSrc,45.0,1.0);
            imageOut = Bitmap.createBitmap(rotated.cols(), rotated.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rotated, imageOut, true);
            saveFile(name, "_Rotate45", imageOut);
            promise.resolve(true);
        } catch (Exception e) {
            promise.reject("100", e.getMessage());
        }
    }

    @ReactMethod
    public void RotateImageAngle(String imagePath, double angle, Promise promise) {
        try {
            String path = Uri.parse(imagePath).getEncodedPath();
            Mat dataSrc = Imgcodecs.imread(path, Imgcodecs.IMREAD_COLOR);
            Bitmap imageSrc = BitmapFactory.decodeFile(path);
            if (imageSrc == null)
                imageSrc = Bitmap.createBitmap(dataSrc.cols(), dataSrc.rows(), Bitmap.Config.ARGB_8888);
            Mat rotated = rotate(dataSrc, angle, 1.0);
            Bitmap imageOut = Bitmap.createBitmap(rotated.cols(), rotated.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rotated, imageOut, true);
            String[] tmp = path.split("/");
            String name = tmp[tmp.length - 1].replace(".jpg", "");
            saveFile(name, "_Original", imageSrc);
            saveFile(name, "rotateAngle", imageOut);
            promise.resolve(true);
        } catch (Exception e) {
            promise.reject("100", e.getMessage());
        }
    }

    private Mat rotate(Mat dataSrc, double angle, double scale) {
        int witdh = dataSrc.cols();
        int heigth = dataSrc.rows();
        Point center = new Point(witdh / 2, heigth / 2);
        Mat rotSrc = Imgproc.getRotationMatrix2D(center, angle, scale);
        Mat rotated = new Mat();
        Size size = new Size(center);
        Imgproc.warpAffine(dataSrc, rotated, rotSrc, size);
        return rotated;
    }


//    @ReactMethod
//    public void textDetection(String imagePath, Promise promise) {
//        try {
//            String path = Uri.parse(imagePath).getEncodedPath();
//            Mat dataSrc = Imgcodecs.imread(path, Imgcodecs.IMREAD_COLOR);
//            Bitmap imageOut = Bitmap.createBitmap(dataSrc.cols(), dataSrc.rows(), Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(dataSrc, imageOut, true);
//            saveFile("IMREAD_COLOR", "_01", imageOut);
//            Bitmap imageSrc = BitmapFactory.decodeFile(path);
//            Mat dataSrc2 = new Mat();
//            Imgproc.cvtColor(dataSrc, dataSrc2, Imgproc.COLOR_BGR2RGB);
//            int cols = dataSrc2.cols();
//            int rows = dataSrc2.rows();
//            int rat = dataSrc2.height() / rows;
//            Mat resizeSrc = new Mat();
//            Size s = new Size(new Point(rat * cols, dataSrc2.height()));
//            Imgproc.resize(dataSrc2, resizeSrc, s);
//            // Resize and convert to grayscale
//            Mat imgGray = new Mat();
//            Imgproc.cvtColor(resizeSrc, imgGray, Imgproc.COLOR_BGR2GRAY);
//            Utils.matToBitmap(imgGray, imageOut, true);
//            saveFile("COLOR_BGR2GRAY", "_02", imageOut);
//            // Bilateral filter preserv edges
//            Mat imgBila = new Mat();
//            Imgproc.bilateralFilter(imgGray, imgBila, 9, 75, 75);
//            Utils.matToBitmap(imgBila, imageOut, true);
//            saveFile("bilateralFilter", "_03", imageOut);
//            // Create black and white image based on adaptive threshold
//            Mat Adaptive = new Mat();
//            Imgproc.adaptiveThreshold(imgGray, Adaptive, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 115, 4);
//            Utils.matToBitmap(Adaptive, imageOut, true);
//            saveFile("adaptiveThreshold", "_04", imageOut);
//            //Median filter clears small details
//            Mat blur = new Mat();
//            Imgproc.medianBlur(Adaptive, blur, 11);
//            Utils.matToBitmap(blur, imageOut, true);
//            saveFile("medianBlur", "_05", imageOut);
//            //Add black border in case that page is touching an image border
//            Mat border = new Mat();
//            Core.copyMakeBorder(blur, border, 5, 5, 5, 5, Core.BORDER_CONSTANT);
//            //Utils.matToBitmap(border, imageOut, true);
//            //saveFile("copyMakeBorder", "_06", imageOut);
//            //Edges
//            Mat edges = new Mat();
//            Imgproc.Canny(border, edges, 200, 250);
//           // Utils.matToBitmap(edges, imageOut, true);
//           // saveFile("edges", "_07", imageOut);
////            //Getting contours
////            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
////            Mat hierarchy = new Mat();
////            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
////            //Finding contour of biggest rectangle
////            // Otherwise return corners of original image
////            // Don't forget on our 5px border!
////            int height = edges.rows();
////            int width = edges.cols();
////            int MAX_COUNTOUR_AREA = (width - 10) * (height - 10);
////            //Page fill at least half of image, then saving max area found
////            double maxAreaFound = MAX_COUNTOUR_AREA * 0.5;
////            //Saving page contour
////            Point[] points = new Point[]{new Point(5.0, 5.0), new Point(5.0, height - 5), new Point(width - 5, height - 5), new Point(width - 5, 5.0)};
////            MatOfPoint2f pageContour = new MatOfPoint2f(points);
////            double perimeter = 0d;
////            // Go through all contours
////            for (MatOfPoint mp : contours) {
////                //Simplify contour
////                MatOfPoint2f mp2f = new MatOfPoint2f(mp.toArray());
////                perimeter = Imgproc.arcLength(mp2f, true);
////                MatOfPoint2f approxCurve = new MatOfPoint2f();
////                Imgproc.approxPolyDP(mp2f, approxCurve, 0.03 * perimeter, true);
////                //Page has 4 corners and it is convex
////                //Page area must be bigger than maxAreaFound
////                double contourArea = Imgproc.contourArea(approxCurve);
////                if (approxCurve.elemSize() == 4 &&
////                        Imgproc.isContourConvex(mp) &&
////                        maxAreaFound < contourArea &&
////                        contourArea < MAX_COUNTOUR_AREA) {
////                    maxAreaFound = contourArea;
////                    pageContour = approxCurve;
////                }
////            }
////            //Result in pageConoutr (numpy array of 4 points)
////
////
////            //Recalculate to original scale - start Points
////            //sPoints = pageContour.dot(image.shape[0] / 800)
////            double sPoints = pageContour.dot(dataSrc2);
////            //Using Euclidean distance
////            //Calculate maximum height (maximal length of vertical edges) and width
////
////            //Create target points
////            //tPoints = np.array([[0, 0],	                    [0, height],	                    [width, height],	                    [width, 0]], np.float32)
////
////            // Mat perspectiveTransform=Imgproc.getPerspectiveTransform(src_mat, dst_mat);
//            Mat perspectiveTransform = new Mat();
//            Imgproc.getPerspectiveTransform(dataSrc2, perspectiveTransform);
//            //Imgproc.warpPerspective()
//            promise.resolve(true);
//        } catch (Exception e) {
//            promise.reject("100", e.getMessage());
//        }
//    }

    double distanceEuclidean(Point p1, Point p2) {
        double result = 0;

        return 0;
    }

    private Mat mGrey, mRgba;

    @ReactMethod
    public void textDetection(String imagePath, Promise promise) {
        try {
            String path = Uri.parse(imagePath).getEncodedPath();
            mRgba = Imgcodecs.imread(path, Imgcodecs.IMREAD_COLOR);
            mGrey= new Mat();
            Imgproc.cvtColor(mRgba, mGrey, Imgproc.COLOR_BGR2GRAY);
            Bitmap imageSrc = BitmapFactory.decodeFile(path);
//            Mat outputImg = recognize(dataSrc);
            detectText();
            Bitmap imageOut = Bitmap.createBitmap(mRgba.cols(), mRgba.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mRgba, imageOut, true);
            String[] tmp = path.split("/");
            String name = tmp[tmp.length - 1].replace(".jpg", "");
            saveFile(name, "_Original", imageSrc);
            saveFile(name, "_TextDetection", imageOut);
            promise.resolve(true);
        } catch (Exception e) {
            promise.reject("100", e.getMessage());
        }
    }
//
//    Scalar RED = new Scalar(255, 0, 0);
//    Scalar GREEN = new Scalar(0, 255, 0);
//    FeatureDetector detector;
//    DescriptorExtractor descriptor;
//    DescriptorMatcher matcher;
//    Mat descriptors2, descriptors1;
//    Mat img1;
//    MatOfKeyPoint keypoints1, keypoints2;
//
//    public Mat recognize(Mat aInputFrame) throws IOException {
//        InitDataDetection();
//        Imgproc.cvtColor(aInputFrame, aInputFrame, Imgproc.COLOR_RGB2GRAY);
//        descriptors2 = new Mat();
//        keypoints2 = new MatOfKeyPoint();
//        detector.detect(aInputFrame, keypoints2);
//        descriptor.compute(aInputFrame, keypoints2, descriptors2);
//
//        // Matching
//        MatOfDMatch matches = new MatOfDMatch();
//        if (img1.type() == aInputFrame.type()) {
//            matcher.match(descriptors1, descriptors2, matches);
//        } else {
//            return aInputFrame;
//        }
//        List<DMatch> matchesList = matches.toList();
//
//        Double max_dist = 0.0;
//        Double min_dist = 100.0;
//
//        for (int i = 0; i < matchesList.size(); i++) {
//            Double dist = (double) matchesList.get(i).distance;
//            if (dist < min_dist)
//                min_dist = dist;
//            if (dist > max_dist)
//                max_dist = dist;
//        }
//
//        LinkedList<DMatch> good_matches = new LinkedList<DMatch>();
//        for (int i = 0; i < matchesList.size(); i++) {
//            if (matchesList.get(i).distance <= (1.5 * min_dist))
//                good_matches.addLast(matchesList.get(i));
//        }
//
//        MatOfDMatch goodMatches = new MatOfDMatch();
//        goodMatches.fromList(good_matches);
//        Mat outputImg = new Mat();
//        MatOfByte drawnMatches = new MatOfByte();
//        if (aInputFrame.empty() || aInputFrame.cols() < 1 || aInputFrame.rows() < 1) {
//            return aInputFrame;
//        }
//        Features2d.drawMatches(img1, keypoints1, aInputFrame, keypoints2, goodMatches, outputImg, GREEN, RED, drawnMatches, Features2d.NOT_DRAW_SINGLE_POINTS);
//        Imgproc.resize(outputImg, outputImg, aInputFrame.size());
//
//        return outputImg;
//    }
//
//    private void InitDataDetection() throws IOException {
//        detector = FeatureDetector.create(FeatureDetector.ORB);
//        descriptor = DescriptorExtractor.create(DescriptorExtractor.ORB);
//        matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
//        img1 = new Mat();
//        AssetManager assetManager = this.reactContext.getAssets();
//        InputStream istr = assetManager.open("b.jpg");
//        Bitmap bitmap = BitmapFactory.decodeStream(istr);
//        Utils.bitmapToMat(bitmap, img1);
//        Imgproc.cvtColor(img1, img1, Imgproc.COLOR_RGB2GRAY);
//        img1.convertTo(img1, 0); //converting the image to match with the type of the cameras image
//        descriptors1 = new Mat();
//        keypoints1 = new MatOfKeyPoint();
//        detector.detect(img1, keypoints1);
//        descriptor.compute(img1, keypoints1, descriptors1);
//    }


    private void detectText() {
        Scalar CONTOUR_COLOR = new Scalar(255);
        MatOfKeyPoint keypoint = new MatOfKeyPoint();
        List<KeyPoint> listpoint;
        KeyPoint kpoint;
        Mat mask = Mat.zeros(mGrey.size(), CvType.CV_8UC1);
        int rectanx1;
        int rectany1;
        int rectanx2;
        int rectany2;
        int imgsize = mGrey.height() * mGrey.width();
        Scalar zeos = new Scalar(0, 0, 0);

        List<MatOfPoint> contour2 = new ArrayList<MatOfPoint>();
        Mat kernel = new Mat(1, 50, CvType.CV_8UC1, Scalar.all(255));
        Mat morbyte = new Mat();
        Mat hierarchy = new Mat();

        Rect rectan3;
        //
        FeatureDetector detector = FeatureDetector
                .create(FeatureDetector.MSER);
        detector.detect(mGrey, keypoint);
        listpoint = keypoint.toList();
        //
        for (int ind = 0; ind < listpoint.size(); ind++) {
            kpoint = listpoint.get(ind);
            rectanx1 = (int) (kpoint.pt.x - 0.5 * kpoint.size);
            rectany1 = (int) (kpoint.pt.y - 0.5 * kpoint.size);
            rectanx2 = (int) (kpoint.size);
            rectany2 = (int) (kpoint.size);
            if (rectanx1 <= 0)
                rectanx1 = 1;
            if (rectany1 <= 0)
                rectany1 = 1;
            if ((rectanx1 + rectanx2) > mGrey.width())
                rectanx2 = mGrey.width() - rectanx1;
            if ((rectany1 + rectany2) > mGrey.height())
                rectany2 = mGrey.height() - rectany1;
            Rect rectant = new Rect(rectanx1, rectany1, rectanx2, rectany2);
            try {
                Mat roi = new Mat(mask, rectant);
                roi.setTo(CONTOUR_COLOR);
            } catch (Exception ex) {
                Log.d("mylog", "mat roi error " + ex.getMessage());
            }
        }
        Imgproc.morphologyEx(mask, morbyte, Imgproc.MORPH_DILATE, kernel);
        Imgproc.findContours(morbyte, contour2, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        for (int ind = 0; ind < contour2.size(); ind++) {
            rectan3 = Imgproc.boundingRect(contour2.get(ind));
            rectan3 = Imgproc.boundingRect(contour2.get(ind));
            if (rectan3.area() > 0.5 * imgsize || rectan3.area() < 100
                    || rectan3.width / rectan3.height < 2) {
                Mat roi = new Mat(morbyte, rectan3);
                roi.setTo(zeos);

            } else
                Imgproc.rectangle(mRgba, rectan3.br(), rectan3.tl(),
                        CONTOUR_COLOR);
        }
    }

    private void saveFile(String name, String tag, Bitmap imageOut) {
        try {
            String file_path = Environment.getExternalStorageDirectory().getAbsolutePath() +
                    "/ImagesOpenCV";
            File dir = new File(file_path);
            if (!dir.exists())
                dir.mkdirs();
            File file = new File(dir, name + tag + ".png");
            FileOutputStream fOut = new FileOutputStream(file);
            imageOut.compress(Bitmap.CompressFormat.PNG, 100, fOut);
            fOut.flush();
            fOut.close();
        } catch (Exception e) {
        }
    }
}
