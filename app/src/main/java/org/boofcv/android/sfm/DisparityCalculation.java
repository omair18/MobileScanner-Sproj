package org.boofcv.android.sfm;
import android.content.Context;
import android.os.Environment;
import android.os.Message;
import android.util.Log;
import android.view.View;

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
import boofcv.gui.image.ShowImages;
import boofcv.gui.image.VisualizeImageData;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.fitting.se.ModelManagerSe3_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.point.Point3D_F64;
import static org.ejml.ops.CommonOps.transpose;
import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
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

	FastQueue<Desc> listSrc;
	FastQueue<Desc> listDst;
	FastQueue<Point2D_F64> locationSrc = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
	FastQueue<Point2D_F64> locationDst = new FastQueue<Point2D_F64>(Point2D_F64.class,true);

	List<AssociatedPair> inliersPixel;

	boolean directionLeftToRight;

	GrayF32 distortedLeft;
	GrayF32 distortedRight;
	GrayF32 rectifiedLeft;
	GrayF32 rectifiedRight;

	// Laplacian that has been applied to rectified images
	GrayF32 edgeLeft;
	GrayF32 edgeRight;

	// has the disparity been computed
	boolean computedDisparity = false;



	private CustomThreadPoolManager mCustomThreadPoolManager;

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

		Se3_F64 leftToRight = estimateCameraMotion(pairs); //estimate rotation & translation from image

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
	public void computeDisparity() {
		if( disparityAlg == null )
			return;
		disparityAlg.process(edgeLeft, edgeRight); // input images after appliying laplacian, to disparity algorithm
		computedDisparity = true;
		//Calling the point cloud function here
		long start = System.currentTimeMillis( );
		Log.d("timeMe", "Entering disparity to point cloud");
		disparityToPointCloud(rectified_pass_mat, r_matrix, t_matrix);
		Log.d("timeMe", "Successfully leaving disparity to point cloud");
		long end = System.currentTimeMillis();
		long diff = end - start;
		Log.d("pcloud", "Time for point cloud processing: " + Long.toString(diff));
		my_file_write(data, "mine.txt");
	}

	public void disparityToPointCloud(DenseMatrix64F rectified_pass_mat, DenseMatrix64F r_matrix, Vector3D_F64 t_matrix) {
		//The point cloud will be in the left cameras reference frame
		//DMatrixRMaj rectK = rectAlg.getCalibrationMatrix();
		//DMatrixRMaj rectR = rectAlg.getRectifiedRotation();
		//estimation of the baseline/distance between two camera centers
		//DenseMatrix64F r_inverse = null;

		final DenseMatrix64F r_inverse = r_matrix;
		final DenseMatrix64F rMatrix = r_matrix;
		transpose(r_matrix, r_inverse); //why do this step?
		Log.d("check123", "This is R inverse/Rectified Rotation? : " + r_inverse.toString() + " and this is" +
				"the rotation: " + r_matrix.toString());

		final double[] result = new double[3]; //baseline calculation formula 1
		result[0] = r_inverse.get(0, 0) * t_matrix.x + r_inverse.get(0, 1) * t_matrix.y + r_inverse.get(0, 2) * t_matrix.z;
		result[1] = r_inverse.get(1, 0) * t_matrix.x + r_inverse.get(1, 1) * t_matrix.y + r_inverse.get(1, 2) * t_matrix.z;
		result[2] = r_inverse.get(2, 0) * t_matrix.x + r_inverse.get(2, 1) * t_matrix.y + r_inverse.get(2, 2) * t_matrix.z;

		final double baseline = Math.sqrt(result[0] * result[0] + result[1] * result[1] + result[2] * result[2]); //baseline formula 2
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

/* for multi threading */
		mCustomThreadPoolManager = CustomThreadPoolManager.getsInstance();
		THREADS_DONE = 0; //a flag to check if threads have done their job or not
		for (int k=0; k<4; k++) {
			CustomRunnable runnable = new CustomRunnable();
			CustomCallable callable = new CustomCallable();
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
			callable.setter(disparityAlg.getDisparity() ,rangeDisparity,disparityAlg.getMinDisparity(), x_0, y_0, baseline, fx, fy, cx, cy, r_matrix, i, j, k, flag);
            callable.setCustomThreadPoolManager(mCustomThreadPoolManager);
            mCustomThreadPoolManager.addCallable(callable);
		}

		if(THREADS_DONE >= 3) {

			Log.d("timeMe", "\n" + "\n" + "Parallel processing ENDED!");
			DisparityActivity.ptCloudBtn.setVisibility(View.VISIBLE);
		}

		//multi threading goes here. Make 4 threads and divide the diparity image
/*		final int x_0 = getDisparity().getWidth();
		final int y_0 = getDisparity().getHeight();
		ExecutorService es = Executors.newCachedThreadPool();
		int f1 = 0, f2 = 0, f3 = 0, f4 = 0;
		es.execute(new Runnable() {
			@Override
			public void run() {
				int idx_1 = 0;
				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
				long startTime = System.currentTimeMillis();

				for (int i = 0; i <= x_0 / 2 - 1; i++) {
					for (int j = 0; j <= y_0 / 2 - 1; j++) {
//						synchronized (this) {
							double d = getDisparity().unsafe_get(i, j) + disparityAlg.getMinDisparity();
							// skip over pixels were no correspondence was found
							idx_1++;
//							if (d >= rangeDisparity)
//								continue;

							// Coordinate in rectified camera frame
							pointRect.z = baseline * fx / d;
							pointRect.x = pointRect.z * (i - cx) / fx;
							pointRect.y = pointRect.z * (j - cy) / fy;

							// rotate into the original left camera frame.
							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
							mat_parallel += "Thread 1: " +  Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";

//						}
					}
				}
				Log.d("timeMe", idx_1 + " Thread " + Thread.currentThread().getId() + " has ended?");
				long endTime = System.currentTimeMillis();
				Log.d("timeMe", "Time for parallel for Thread: " + Thread.currentThread().getId() + " " + Long.toString(endTime - startTime));
//				f1 = 1;
			}
		});

		es.execute(new Runnable() {
			@Override
			public void run() {
				int idx_2 = 0;
				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
				long startTime = System.currentTimeMillis();
//				synchronized (this) {
					for (int i = x_0 / 2; i <= x_0; i++) {
						for (int j = 0; j <= y_0 / 2 - 1; j++) {
							double d = getDisparity().unsafe_get(i, j) + disparityAlg.getMinDisparity();
							idx_2++;
							// skip over pixels were no correspondence was found
							if (d >= rangeDisparity)
								continue;

							// Coordinate in rectified camera frame
							pointRect.z = baseline * fx / d;
							pointRect.x = pointRect.z * (i - cx) / fx;
							pointRect.y = pointRect.z * (j - cy) / fy;

							// rotate into the original left camera frame.
							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
							mat_parallel += "Thread 2: " + Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
						}
					}
//				}
				Log.d("timeMe", idx_2 + " Thread " + Thread.currentThread().getId() + " has ended?");
				long endTime = System.currentTimeMillis();
				Log.d("timeMe", " Time for parallel for Thread: " + Thread.currentThread().getId() + " " + Long.toString(endTime - startTime));

			}
		});

		es.execute(new Runnable() {
			@Override
			public void run() {
				int idx_3 = 0;
				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
				long startTime = System.currentTimeMillis();

//				synchronized (this) {
					for (int i = 0; i <= x_0/2 - 1 ; i++) {
						for(int j = y_0/2; j <= y_0 - 1; j++) {
							double d = getDisparity().unsafe_get(i,j) + disparityAlg.getMinDisparity();
							// skip over pixels were no correspondence was found
							if( d >= rangeDisparity )
								continue;

							// Coordinate in rectified camera frame
							pointRect.z = baseline*fx/d;
							pointRect.x = pointRect.z*(i - cx)/fx;
							pointRect.y = pointRect.z*(j - cy)/fy;

							// rotate into the original left camera frame.
							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
							mat_parallel += "Thread 3: " +
									Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
							idx_3++;
						}
					}
//				}

				Log.d("timeMe", idx_3 + " Thread " + Thread.currentThread().getId() + " has ended?");
				long endTime = System.currentTimeMillis();
				Log.d("timeMe" , "Time for parallel for Thread: " + Thread.currentThread().getId() + " " +  Long.toString(endTime - startTime));
			}
		});
		es.shutdown();*/

//		int o = 0;
//		while(o < 1000000) {
//			o++;
//		}
//		try {
//			boolean finished = es.awaitTermination(10, TimeUnit.MINUTES);
//		} catch (InterruptedException io) {
//			Log.d("timeMe", io.getMessage() + " " + "Stack trace: ");
//		}
//		Thread[] threads = new Thread[2];
//		//thread 1 goes from (0 , 0) to (x_0/2 - 1, y_0/2 - 1)
//		threads[0] = new Thread(new Runnable() {
//			@Override
//			public void run() {
//				int idx_1 = 0;
//				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
//				long startTime = System.currentTimeMillis();
//
//				for (int i = 0; i <= x_0/2 - 1  ; i++) {
//					for(int j = 0; j <= y_0/2 - 1; j++) {
//						synchronized (this) {
//							double d = getDisparity().unsafe_get(i,j) + disparityAlg.getMinDisparity();
//							// skip over pixels were no correspondence was found
//							idx_1++;
//							if( d >= rangeDisparity )
//								continue;
//
//							// Coordinate in rectified camera frame
//							pointRect.z = baseline*fx/d;
//							pointRect.x = pointRect.z*(i - cx)/fx;
//							pointRect.y = pointRect.z*(j - cy)/fy;
//
//							// rotate into the original left camera frame.
//							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
//							mat_parallel += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
//
//						}
//					}
//				}
//				Log.d("timeMe", idx_1 + " Thread " + Thread.currentThread().getId() + " has ended?");
//				long endTime = System.currentTimeMillis();
//				Log.d("timeMe" , "Time for parallel for Thread: " + Thread.currentThread().getId() + " " +  Long.toString(endTime - startTime));
//			}
//		});//.start();
//
//		//thread 2 goes from (x_0/2 , 0) to (x_0 , y_0/2 - 1)
//		threads[1] =  new Thread(new Runnable() {
//			@Override
//			public void run() {
//				int idx_2 = 0;
//				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
//				long startTime = System.currentTimeMillis();
//				synchronized (this) {
//					for (int i = x_0/2; i <= x_0  ; i++) {
//						for(int j = 0; j <= y_0/2 - 1; j++) {
//							double d = getDisparity().unsafe_get(i,j) + disparityAlg.getMinDisparity();
//							idx_2++;
//							// skip over pixels were no correspondence was found
//							if( d >= rangeDisparity )
//								continue;
//
//							// Coordinate in rectified camera frame
//							pointRect.z = baseline*fx/d;
//							pointRect.x = pointRect.z*(i - cx)/fx;
//							pointRect.y = pointRect.z*(j - cy)/fy;
//
//							// rotate into the original left camera frame.
//							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
//							mat_parallel += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
//						}
//					}
//				}
//				Log.d("timeMe", idx_2 + " Thread " + Thread.currentThread().getId() + " has ended?");
//				long endTime = System.currentTimeMillis();
//				Log.d("timeMe" , " Time for parallel for Thread: " + Thread.currentThread().getId() + " " +  Long.toString(endTime - startTime));
//
//		}); //.start();

//		//thread 3 goes from (0 , y_0/2-1) to (x_0/2 - 1, y_0/2)
//		threads[2] = new Thread(new Runnable() {
//			@Override
//			public void run() {
//				int idx_3 = 0;
//				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
//				long startTime = System.currentTimeMillis();
//
//				synchronized (this) {
//					for (int i = 0; i <= x_0/2 - 1 ; i++) {
//						for(int j = y_0/2; j <= y_0; j++) {
//							double d = getDisparity().unsafe_get(i,j) + disparityAlg.getMinDisparity();
//							// skip over pixels were no correspondence was found
//							if( d >= rangeDisparity )
//								continue;
//
//							// Coordinate in rectified camera frame
//							pointRect.z = baseline*fx/d;
//							pointRect.x = pointRect.z*(i - cx)/fx;
//							pointRect.y = pointRect.z*(j - cy)/fy;
//
//							// rotate into the original left camera frame.
//							GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
//							mat_parallel += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
//							idx_3++;
//						}
//					}
//				}
//
//				Log.d("timeMe", idx_3 + " Thread " + Thread.currentThread().getId() + " has ended?");
//				long endTime = System.currentTimeMillis();
//				Log.d("timeMe" , "Time for parallel for Thread: " + Thread.currentThread().getId() + " " +  Long.toString(endTime - startTime));
//
//			}
//		});//.start();
//
//		for(int i = 0; i <= 1; i++)
//			threads[i].start();
//
//		for (Thread t : threads) {
//			try {
//				Log.d("timeMe", "Joining thread is:" + t.getId());
//				t.join();
//				Log.d("timeMe", "Thread: " + t.getId() + " has successfully joined");
//			} catch (InterruptedException e) {
//				Log.d("timeMe", "Try catch 0: " + e.toString() + " " +
//					e.getLocalizedMessage());
//			}
//		}
//		for (int i = 0 ; i <= 1; i++) {
//			threads[i].join();
//		}

//		//thread 4 goes from (x_0/2 - 1, y_0/2 - 1) to (x_0 , y_0)
//		new Thread(new Runnable() {
//			@Override
//			public void run() {
//				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has begun");
//				long startTime = System.currentTimeMillis();
//
//				for (int i = x_0/2; i <= x_0  ; i++) {
//					for(int j = y_0/2; j <= y_0; j++) {
//						double d = getDisparity().unsafe_get(i,j) + disparityAlg.getMinDisparity();
//						// skip over pixels were no correspondence was found
//						if( d >= rangeDisparity )
//							continue;
//
//						// Coordinate in rectified camera frame
//						pointRect.z = baseline*fx/d;
//						pointRect.x = pointRect.z*(i - cx)/fx;
//						pointRect.y = pointRect.z*(j - cy)/fy;
//
//						// rotate into the original left camera frame.
//						GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
//						mat_parallel += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + ";";
//					}
//				}
//				Log.d("timeMe", "Thread " + Thread.currentThread().getId() + " has ended?");
//				long endTime = System.currentTimeMillis();
//				Log.d("timeMe" , "Time for parallel for Thread: " + Thread.currentThread().getId() + " " +  Long.toString(endTime - startTime));
//			}
//		}).start();
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

	public void my_file_write(String data, String fileName){
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
	}

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