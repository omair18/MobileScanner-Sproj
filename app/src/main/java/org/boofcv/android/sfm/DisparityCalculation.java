package org.boofcv.android.sfm;
import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.os.Build;
import android.os.Environment;
import android.os.Message;
import android.support.annotation.RequiresApi;
import android.util.Log;
import android.util.SizeF;
import android.view.View;

import org.boofcv.android.camera2basic.Camera2BasicFragment;
import org.ddogleg.fitting.modelset.DistanceFromModel;
import org.ddogleg.fitting.modelset.ModelGenerator;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;
import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;

import java.io.BufferedWriter;
import java.lang.Object.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;
import boofcv.gui.d3.DisparityPointCloudViewer;
import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.disparity.StereoDisparity;
import boofcv.abst.geo.Estimate1ofEpipolar;
import boofcv.abst.geo.TriangulateTwoViewsCalibrated;
import boofcv.alg.depth.VisualDepthOps;
import boofcv.alg.descriptor.UtilFeature;
import boofcv.alg.distort.ImageDistort;
import boofcv.alg.distort.LensDistortionOps;
import boofcv.alg.filter.derivative.LaplacianEdge;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.RectifyImageOps;
import boofcv.alg.geo.rectify.RectifyCalibrated;
import boofcv.alg.geo.robust.DistanceSe3SymmetricSq;
import boofcv.alg.geo.robust.Se3FromEssentialGenerator;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.core.image.border.BorderType;
import boofcv.factory.geo.EnumEpipolar;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.gui.d3.PointCloudTiltPanel;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.gui.image.ShowImages;
import boofcv.gui.image.VisualizeImageData;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.distort.DoNothing2Transform2_F64;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import georegression.fitting.se.ModelManagerSe3_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.point.Point3D_F64;

import static boofcv.android.ConvertBitmap.grayToBitmap;
import static java.lang.Math.tan;
import static org.ejml.ops.CommonOps.transpose;
import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Computes the disparity image from two views. Features are first associated between the two images, image motion
 * found, rectification and dense stereo calculation.
 *
 * @author Peter Abeles
 */
public class DisparityCalculation<Desc extends TupleDesc> {
	DetectDescribePoint<GrayF32,Desc> detDesc;
	AssociateDescription<Desc> associate;
	CameraPinholeRadial intrinsic;
	//Camera mCamera;

	StereoDisparity<GrayF32, GrayF32> disparityAlg;
	//Variables for storing the point cloud
	public String data = "";
    public String mat = "[";
	public String mat_parallel = "[";
    //	The Rectified Calibration, rotation matrices and the translation vector have been defined in a global scope.
    public DenseMatrix64F rectified_pass_mat, r_matrix;
    public Vector3D_F64 t_matrix;
    public String path = Environment.getExternalStorageDirectory().getAbsolutePath();
	public String path_public = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).getAbsolutePath();

	public static int THREADS_DONE = 0;
	public Se3_F64 leftToRight;

	FastQueue<Desc> listSrc;
	FastQueue<Desc> listDst;
	FastQueue<Point2D_F64> locationSrc = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
	FastQueue<Point2D_F64> locationDst = new FastQueue<Point2D_F64>(Point2D_F64.class,true);

	List<AssociatedPair> inliersPixel;

	boolean directionLeftToRight;

	public GrayF32 distortedLeft;
	public GrayF32 distortedRight;
	public GrayF32 rectifiedLeft;
	public GrayF32 rectifiedRight;
	public Bitmap colorimage;

	// Laplacian that has been applied to rectified images
	GrayF32 edgeLeft;
	GrayF32 edgeRight;

	// has the disparity been computed
	boolean computedDisparity = false;

	public static Semaphore[] semaphores = new Semaphore[4]; // 4 threads, so 4 semaphores


    private void initSemaphores() {
		semaphores[0] = new Semaphore(1);
		semaphores[1] = new Semaphore(0);
		semaphores[2] = new Semaphore(0);
		semaphores[3] = new Semaphore(0);
	}


	private static CustomThreadPoolManager mCustomThreadPoolManager;

	public DisparityCalculation(DetectDescribePoint<GrayF32, Desc> detDesc,
								AssociateDescription<Desc> associate ,
								CameraPinholeRadial intrinsic ) {
		this.detDesc = detDesc;
		this.associate = associate;
		this.intrinsic = intrinsic;

		listSrc = UtilFeature.createQueue(detDesc, 10);
		listDst = UtilFeature.createQueue(detDesc, 10);
	}

	public void setDisparityAlg(StereoDisparity<GrayF32, GrayF32> disparityAlg) {
		this.disparityAlg = disparityAlg;
	}

	public void init( int width , int height ) {
		distortedLeft = new GrayF32(width,height);
		distortedRight = new GrayF32(width,height);
		rectifiedLeft = new GrayF32(width,height);
		rectifiedRight = new GrayF32(width,height);
		edgeLeft = new GrayF32(width,height);
		edgeRight = new GrayF32(width,height);
	}

	public void setSource( GrayF32 image ) {
		distortedLeft.setTo(image);

		long start = System.currentTimeMillis( );
		detDesc.detect(image);
		describeImage(listSrc, locationSrc); // compute descriptors
		long end = System.currentTimeMillis( );
		long diff = end - start;
		Log.d("detection", Float.toString(diff));

		associate.setSource(listSrc);
	}
	public static void resetEverything() {
	    /*if(listSrc != null)
	    	listSrc.reset();
		if(listDst != null)
	    	listDst.reset();
		if(locationDst != null)
	    	locationDst.reset();
		if(locationSrc != null)
	    	locationSrc.reset();
	    if(inliersPixel != null);
	    	inliersPixel.clear(); */
	    if(mCustomThreadPoolManager != null)
	    	mCustomThreadPoolManager.cancelAllTasks();

	}
	public void setDestination( GrayF32 image ) {
		distortedRight.setTo(image);
		detDesc.detect(image);
		describeImage(listDst, locationDst); //compute descriptors
		associate.setDestination(listDst);

	}

	private void describeImage(FastQueue<Desc> listDesc, FastQueue<Point2D_F64> listLoc) { //descriptors, location
		listDesc.reset();
		listLoc.reset();
		int N = detDesc.getNumberOfFeatures();
		for( int i = 0; i < N; i++ ) {
			listLoc.grow().set(detDesc.getLocation(i));
			listDesc.grow().setTo(detDesc.getDescription(i));
		}
	}
	/**
	 * Associates image features, computes camera motion, and rectifies images.
	 *
	 * @return true it was able to rectify the input images or false if not
	 */
	public boolean rectifyImage() {
		computedDisparity = false;

		associate.associate(); //find the best matches
		List<AssociatedPair> pairs = convertToNormalizedCoordinates();

		leftToRight = estimateCameraMotion(pairs); //estimate rotation & translation from image

		if( leftToRight == null ) {
			Log.e("disparity","estimate motion failed");
			Log.e("disparity","  left.size = "+locationSrc.size());
			Log.e("disparity","  right.size = "+locationDst.size());
			Log.e("disparity", "  associated size = " + associate.getMatches().size());
			Log.e("disparity","  pairs.size = "+pairs.size());

			return false;
		} else if( leftToRight.getT().x > 0 ) {
			// the user took a picture from right to left instead of left to right
			// so now everything needs to be swapped
			leftToRight = leftToRight.invert(null);
			GrayF32 tmp = distortedLeft;
			distortedLeft = distortedRight;
			distortedRight = tmp;
			tmp = edgeLeft;
			edgeLeft = edgeRight;
			edgeRight = tmp;
			directionLeftToRight = false;
		} else {
			directionLeftToRight = true;
		}

		DenseMatrix64F rectifiedK = new DenseMatrix64F(3,3); //K matrix? I think so!
		Log.d("rectify","time to estimate");
		long start = System.currentTimeMillis();
		Log.d("rectify","Time has begun");
        rectifyImages(leftToRight, rectifiedK); // at this point rectifiedK is empty
        long end = System.currentTimeMillis( );
		long diff = end - start;
		Log.d("rectify","Time to rectify: " + diff);

		rectified_pass_mat = rectifiedK;
		r_matrix = leftToRight.getR(); //get rotation
		t_matrix = leftToRight.getT(); //get translation
		Log.d("check123", "This is rectified pass, r and t: " +rectified_pass_mat.toString() + "\n" + r_matrix.toString() + t_matrix.toString());
;		return true;
	}
//	//	The Rectified Calibration, rotation matrices and the translation vector have been defined in a global scope.
//	public DenseMatrix64F rectified_pass_mat, r_matrix;
//	public Vector3D_F64 t_matrix;

	/**.
	 * Computes the disparity between the two rectified images
	 */
	@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
	public void computeDisparity() {
		if( disparityAlg == null )
			return;
		disparityAlg.process(edgeLeft, edgeRight); // input images after appliying laplacian, to disparity algorithm
		computedDisparity = true;
		//Calling the point cloud function here
		long start = System.currentTimeMillis( );
		Log.d("timeMe", "Entering disparity to point cloud");

		//leftimage = grayToBitmap(rectifiedLeft, Bitmap.Config.ARGB_8888);

		disparityToPointCloud(rectified_pass_mat, r_matrix, t_matrix, colorimage);
		//newdisparityToPointCloud();
		Log.d("timeMe", "Successfully leaving disparity to point cloud");
		long end = System.currentTimeMillis();
		long diff = end - start;
		Log.d("pcloud", "Time for point cloud processing: " + Long.toString(diff));
		//my_file_write(data, "mine.txt");
	}

	@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
	public void disparityToPointCloud(DenseMatrix64F rectified_pass_mat, DenseMatrix64F r_matrix, Vector3D_F64 t_matrix, Bitmap colortimage) {
		//The point cloud will be in the left cameras reference frame
		//DMatrixRMaj rectK = rectAlg.getCalibrationMatrix();
		//DMatrixRMaj rectR = rectAlg.getRectifiedRotation();
		//estimation of the baseline/distance between two camera centers
		//DenseMatrix64F r_inverse = null;
		final DenseMatrix64F r_inverse = r_matrix;
		final DenseMatrix64F rMatrix = r_matrix;
		//transpose(r_matrix, r_inverse); //why do this step?
		Log.d("check123", "This is R inverse/Rectified Rotation? : " + r_inverse.toString() + " and this is" +
				"the rotation: " + r_matrix.toString());

		final double[] result = new double[3]; //baseline calculation formula 1
		result[0] = r_inverse.get(0, 0) * t_matrix.x + r_inverse.get(0, 1) * t_matrix.y + r_inverse.get(0, 2) * t_matrix.z;
		result[1] = r_inverse.get(1, 0) * t_matrix.x + r_inverse.get(1, 1) * t_matrix.y + r_inverse.get(1, 2) * t_matrix.z;
		result[2] = r_inverse.get(2, 0) * t_matrix.x + r_inverse.get(2, 1) * t_matrix.y + r_inverse.get(2, 2) * t_matrix.z;

		//final double baseline = Math.sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2]); //baseline formula 2
		final double baseline = leftToRight.getT().norm();
		float sensor_width = Camera2BasicFragment.sensor_width;
		float sensor_height = Camera2BasicFragment.sensor_height;
		float focal_length = Camera2BasicFragment.focal_length;

		Log.d("check123", "This is the baseline: " + baseline + "\n and " +
				"this is the translation:" + t_matrix.toString());

		// extract intrinsic parameters from the rectified camera
		final double fx = rectified_pass_mat.get(0, 0); //
		//double fx = rectK.get(0,0);
		final double fy = rectified_pass_mat.get(1, 1);
		//double fy = rectK.get(1,1);
		final double cx = rectified_pass_mat.get(0, 2);
		//double cx = rectK.get(0,2);
		final double cy = rectified_pass_mat.get(1, 2);
		//double cy = rectK.get(1,2);

		// Iterate through each pixel in disparity image and compute its 3D coordinate
		final Point3D_F64 pointRect = new Point3D_F64();
		final Point3D_F64 pointLeft = new Point3D_F64();
		long currentTime = System.currentTimeMillis();

		final int rangeDisparity = disparityAlg.getMaxDisparity() - disparityAlg.getMinDisparity();
		int idx = 0;
		Log.d("timeMe", "\n" + "\n" + "Parallel processing begins!" +
				"\n" + "\n");

		int x_0 = getDisparity().getWidth();
		int y_0 = getDisparity().getHeight();

		initSemaphores();

/* for multi threading */
		mCustomThreadPoolManager = CustomThreadPoolManager.getsInstance();
		THREADS_DONE = 0; //a flag to check if threads have done their job or not
		for (int k=0; k<4; k++) {
			CustomRunnable runnable = new CustomRunnable();
			CustomCallable callable = new CustomCallable(semaphores,k);
			//setter for each thread, each thread will take some values and perform its own calculation. Once calculation is done, it writes all points in a file.ply
			boolean flag = true;
			int i=0,j=0;
			if(k == 0) { //if first thread  /// suppose we have width range = 0-10, height range = 0- 10
				i=0;
				x_0 = getDisparity().getWidth()/2 -1; // 0 t0 4
				j=0;
				y_0 = getDisparity().getHeight()/2 -1; // 0 to 4
			}
			else if (k == 1) { // if 2nd thread
				i = getDisparity().getWidth()/2; //
				x_0 = getDisparity().getWidth()-1; // 5 to 9
				j = 0;
				y_0 = getDisparity().getHeight()/2 -1; // 0 to 4
			}
			else if (k == 2) { // if 3rd thread
				i=0;
				x_0 = getDisparity().getWidth()/2 -1; // 0 to 4
				j= getDisparity().getHeight()/2;
				y_0 = getDisparity().getHeight()-1; // 5 to 9
			}
			else { // if 4th thread
				i = getDisparity().getWidth()/2; // 5 to 9
				x_0 = getDisparity().getWidth()-1;
				j= getDisparity().getHeight()/2;
				y_0 = getDisparity().getHeight()-1; // 5 to 9

			}
			callable.setter(disparityAlg.getDisparity() ,rangeDisparity,disparityAlg.getMinDisparity(), x_0, y_0, baseline, fx, fy, cx, cy, r_matrix, sensor_width, sensor_height, focal_length, i, j, k, flag, colorimage);
            callable.setCustomThreadPoolManager(mCustomThreadPoolManager);
            mCustomThreadPoolManager.addCallable(callable);
		}

		if(THREADS_DONE >= 3) {

			Log.d("timeMe", "\n" + "\n" + "Parallel processing ENDED!");
			//DisparityActivity.ptCloudBtn.setVisibility(View.VISIBLE);
		}
			Log.d("timeMe", "Returning from disparityto3D");
	}

	public static void appendLog(String text, String fileName)
	{
		File logFile = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
		Log.d("check123", "File details before injection: " + logFile.getAbsolutePath() + "\n" +
			"Does it exist? : " + logFile.isFile() + "\n" +
				"Is it a directory?: " +  logFile.isDirectory() + "\n" +
					"It has a length of: " + logFile.length());
		if (!logFile.exists())
		{
			Log.d("check123", "The file creation failed");
			try
			{
				logFile.createNewFile();
			} catch (IOException e)
			{
				e.printStackTrace();
			}
		}
		try
		{
			Log.d("check123", "The file creation succeeded");
			// BufferedWriter for performance, true to set append to file flag
			BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
			buf.append(text);
			buf.newLine();
			buf.close();
			Log.d("check123", "File details after injection: " + logFile.getAbsolutePath() + "\n" +
					"Does it exist? : " + logFile.isFile() + "\n" +
					"Is it a directory?: " +  logFile.isDirectory() + "\n" +
					"It has a length of: " + logFile.length());
		} catch (IOException e)
		{
			Log.d("check123", "IOException reached: " + e.toString());
			e.printStackTrace();
		}
	}

	/*public void my_file_write(String data, String fileName){
		try {
			Log.d("pcloud", "Entered write operation!");
			String dummy = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss").format(new Date());
			File xmlFile = new File(Environment.getExternalStorageDirectory() + "/" + dummy);
			org.apache.commons.io.FileUtils.writeStringToFile(xmlFile, data, "UTF-8" );
			Log.d("pcloud", Environment.getExternalStorageDirectory().toString());
		} catch (Exception e) {
			Log.d("pcloud","Exception during file writing " +  e.toString());
			e.printStackTrace();
		}
	} */

	/**
	 * Convert a set of associated point features from pixel coordinates into normalized image coordinates.
	 */
	public List<AssociatedPair> convertToNormalizedCoordinates() {

		Point2Transform2_F64 tran = LensDistortionOps.transformPoint(intrinsic).undistort_F64(true,false);

		List<AssociatedPair> calibratedFeatures = new ArrayList<AssociatedPair>();

		FastQueue<AssociatedIndex> matches = associate.getMatches();
		for( AssociatedIndex a : matches.toList() ) {
			Point2D_F64 p1 = locationSrc.get( a.src );
			Point2D_F64 p2 = locationDst.get( a.dst );

			AssociatedPair c = new AssociatedPair();

			tran.compute(p1.x, p1.y, c.p1);
			tran.compute(p2.x, p2.y, c.p2);

			calibratedFeatures.add(c);
		}

		return calibratedFeatures;
	}

	public volatile int numInside = 0;
	/**
	 * Estimates image motion up to a scale factor
	 * Estimates the camera motion robustly using RANSAC and a set of associated points.
	 * @param matchedNorm set of matched point features in normalized image coordinates
	 * @return Found camera motion.  Note translation has an arbitrary scale
	 */
	public Se3_F64 estimateCameraMotion( List<AssociatedPair> matchedNorm )
	{
		long start = System.currentTimeMillis( );

		numInside++;
		System.out.println("DISPARITY "+ numInside);
		Estimate1ofEpipolar essentialAlg = FactoryMultiView.computeFundamental_1(EnumEpipolar.ESSENTIAL_5_NISTER, 5);
		TriangulateTwoViewsCalibrated triangulate = FactoryMultiView.triangulateTwoGeometric();
		ModelGenerator<Se3_F64, AssociatedPair> generateEpipolarMotion =
				new Se3FromEssentialGenerator(essentialAlg, triangulate);

		DistanceFromModel<Se3_F64, AssociatedPair> distanceSe3 =
				new DistanceSe3SymmetricSq(triangulate,
						intrinsic.fx, intrinsic.fy, intrinsic.skew,
						intrinsic.fx, intrinsic.fy, intrinsic.skew);

		// 1/2 a pixel tolerance for RANSAC inliers
		double ransacTOL = 0.5 * 0.5 * 2.0;

		ModelManager<Se3_F64> mm = new ModelManagerSe3_F64();

		ModelMatcher<Se3_F64, AssociatedPair> epipolarMotion =
				new Ransac<Se3_F64, AssociatedPair>(2323, mm,generateEpipolarMotion, distanceSe3,
						300, ransacTOL);

		if (!epipolarMotion.process(matchedNorm)) {
			numInside--;
			return null;
		}

		createInliersList(epipolarMotion);

		numInside--;
		long end = System.currentTimeMillis( );

		return epipolarMotion.getModelParameters();
	}

	/**
	 * Save a list of inliers in pixel coordinates
	 */
	private void createInliersList( ModelMatcher<Se3_F64, AssociatedPair> epipolarMotion ) {
		inliersPixel = new ArrayList<AssociatedPair>();

		FastQueue<AssociatedIndex> matches = associate.getMatches();

		int N = epipolarMotion.getMatchSet().size();
		for( int i = 0; i < N; i++ ) {

			AssociatedIndex a = matches.get( epipolarMotion.getInputIndex(i));

			Point2D_F64 p1 = locationSrc.get( a.src );
			Point2D_F64 p2 = locationDst.get( a.dst );

			inliersPixel.add( new AssociatedPair(p1,p2));
		}
	}
	/**
	 * Remove lens distortion and rectify stereo images
	 *
	 * @param leftToRight    Camera motion from left to right
	 * @param rectifiedK     Output camera calibration matrix for rectified camera
	 */
	public void rectifyImages(Se3_F64 leftToRight,
							  DenseMatrix64F rectifiedK) {
		RectifyCalibrated rectifyAlg = RectifyImageOps.createCalibrated();

        Log.d("check123", "0: intrinsic before" + intrinsic.toString());
        // original camera calibration matrices
		DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);
		Log.d("check123", "1: intrinsic after" + intrinsic.toString());
		rectifyAlg.process(K, new Se3_F64(), K, leftToRight);

		// rectification matrix for each image
		DenseMatrix64F rect1 = rectifyAlg.getRect1();
		DenseMatrix64F rect2 = rectifyAlg.getRect2();

		// New calibration matrix,
		rectifiedK.set(rectifyAlg.getCalibrationMatrix());

		// Adjust the rectification to make the view area more useful
		RectifyImageOps.allInsideLeft(intrinsic, rect1, rect2, rectifiedK); // #############

		// undistorted and rectify images
		ImageDistort<GrayF32,GrayF32> distortLeft =
				RectifyImageOps.rectifyImage(intrinsic, rect1, BorderType.ZERO, ImageType.single(GrayF32.class));
		ImageDistort<GrayF32,GrayF32> distortRight =
				RectifyImageOps.rectifyImage(intrinsic, rect2, BorderType.ZERO, ImageType.single(GrayF32.class));

		// Apply the Laplacian for some lighting invariance
		ImageMiscOps.fill(rectifiedLeft,0);
		distortLeft.apply(distortedLeft, rectifiedLeft);
		LaplacianEdge.process(rectifiedLeft,edgeLeft);

		//fills whole image with specific value, in this case 0
		ImageMiscOps.fill(rectifiedRight, 0);
		distortRight.apply(distortedRight, rectifiedRight);
		LaplacianEdge.process(rectifiedRight,edgeRight);
	}

	public List<AssociatedPair> getInliersPixel() {
		return inliersPixel;
	}

	public GrayF32 getDisparity() {
		return disparityAlg.getDisparity();
	}

	public StereoDisparity<GrayF32, GrayF32> getDisparityAlg() {
		return disparityAlg;
	}

	public boolean isDisparityAvailable() {
		return computedDisparity;
	}

	public boolean isDirectionLeftToRight() {
		return directionLeftToRight;
	}

	/*private WeakReference<CustomThreadPoolManager> mCustomThreadPoolManagerWeakReference;

	@Override
	public Object call() throws Exception {
		try {
			// check if thread is interrupted before lengthy operation
			if (Thread.interrupted()) throw new InterruptedException();

			// In real world project, you might do some blocking IO operation
			// In this example, I just let the thread sleep for 3 second
			Thread.sleep(3000);

			// After work is finished, send a message to CustomThreadPoolManager
			Message message = Util.createMessage(Util.MESSAGE_ID, "Thread " +
					String.valueOf(Thread.currentThread().getId()) + " " +
					String.valueOf(Thread.currentThread().getName()) + " completed");

			if(mCustomThreadPoolManagerWeakReference != null
					&& mCustomThreadPoolManagerWeakReference.get() != null) {

				mCustomThreadPoolManagerWeakReference.get().sendMessageToUiThread(message);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return null;
	}

	public void setCustomThreadPoolManager(CustomThreadPoolManager customThreadPoolManager) {
		this.mCustomThreadPoolManagerWeakReference = new WeakReference<CustomThreadPoolManager>(customThreadPoolManager);
	} */
}