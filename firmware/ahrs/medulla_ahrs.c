#include <medulla_ahrs.h>

/* Some static variables */
static float v_gyr[3];        /* Gyroscope read */
static float v_acc[3];        /* Accelerometer read */
static float v_acc_last[3];   /* Last accelerometer read */
static float acc_scale;       /* Scale down accelerometer weight if it differs from 1 g too much. */
static float acc_weight;      /* Variable accelerometer weight */

//static float k_bb[3];   /* K body unit vector expressed in body coordinates */
static float k_gb[3];   /* K global unit vector expressed in body coordinates */
//static float j_gb[3];   /* J global unit vector expressed in body coordinates */

static float w_a[3];    /* Corrective rotation vector based on acceleration */
//static float w_m[3];    /* Corrective rotation vector based on magnetometer */
static float w_dt[3];   /* Angular displacement vector = w * dt, where w is the angular velocity vector and dt is the time elapsed. */

static float dcm_gyro[3][3];   /* DCM based on gyro readings, corrected with w_a. */
static float trim_angle[3];   /* Euler angles to help trim. TODO: Figure this out properly. */
static float dcm_trim[3][3];   /* DCM used to offset dcm_gyro to produce dcm_bg. */
static float dcm_d[3][3];   /* First used to store the change in DCM to update dcm_bg. Repurposed during orthonormalization to store the correction vectors for the i and j unit vectors. */
static float dcm_err;   /* DCM error for which we need orthonormalization. */

static void orthonormalize(float dcm[3][3])
{
	/* Orthogonalize the i and j unit vectors (DCMDraft2 Eqn. 19). */
	dcm_err = v_dotp(dcm[0], dcm[1]);
	v_scale(dcm[1], -dcm_err/2, dcm_d[0]);   /* i vector correction */
	v_scale(dcm[0], -dcm_err/2, dcm_d[1]);   /* i vector correction */
	v_add(dcm[0], dcm_d[0], dcm[0]);
	v_add(dcm[1], dcm_d[1], dcm[1]);

	/* k = i x j */
	v_crossp(dcm[0], dcm[1], dcm[2]);

	/* Normalize all three vectors. */
	v_norm(dcm[0]);
	v_norm(dcm[1]);
	v_norm(dcm[2]);
}


void setup_ahrs(void)
{
	/* Set up IMU. */
	setup_kvh();

	/* Initialize DCMs as identity matrices. */
	static uint8_t i, j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			m_init_identity(dcm_gyro);
			m_init_identity(dcm_trim);
			m_init_identity(dcm_d);
		}
	}

	/**
	 * Calculate DCM offset.
	 *
	 * We use an accelerometer to correct for gyro drift. This means that the
	 * IMU will adjust dcm_gyro according to whatever the accelerometer thinks
	 * is gravity. However, since it is nearly impossible to mount the
	 * accelerometer perfectly horizontal to the chassis, this also means that
	 * dcm_gyro will represent the orientation of the accelerometer, not the
	 * chassis, along the X and Y rotational axes.
	 *
	 * The rotational difference between the orientations of the accelerometer
	 * (dcm_gyro) and the chassis (dcm_out) is constant and can be approximated
	 * by another rotation matrix we will call dcm_trim, which we populate with
	 * trim angles obtained during hover calibration. We rotate dcm_gyro by
	 * dcm_trim to obtain dcm_out.
	 *
	 * Keep in mind, however, that we still update the IMU based on dcm_gyro and
	 * not dcm_out. This is because it is dcm_gyro we are correcting with the
	 * accelerometer's measurement of the gravity vector, and dcm_out is only
	 * a transformation of that DCM based on dcm_trim. We use dcm_out for
	 * flight calculations, but dcm_gyro is what we keep track of within the
	 * IMU.
	 *
	 * As a final note, this is a small-angle approximation! We could do
	 * something fancy with trigonometry, but if the hardware is so poorly
	 * built (or poorly designed) that small-angle approximations become
	 * insufficient, we can't expect software to fix everything.
	 */
	#ifdef ACC_WEIGHT
	// TODO: Set trim angles.
	//trim_angle[0] = TRIM_ANGLE_X;
	//trim_angle[1] = TRIM_ANGLE_Y;

	v_acc[0] = 0;
	v_acc[1] = 0;
	v_acc[2] = 1;
	for (i=0; i<3; i++) {
		v_acc_last[i] = v_acc[i];
	}
	//accVar = 100.;
	#endif // ACC_WEIGHT
}

void update_ahrs(float dt, float dcm_out[3][3])
{
	static uint8_t i;

	read_kvh(v_gyr, v_acc);

	/**
	 * Accelerometer
	 *     Frame of reference: BODY
	 *     Units: G (gravitational acceleration)
	 *     Purpose: Measure the acceleration vector v_acc with components
	 *              codirectional with the i, j, and k vectors. Note that the
	 *              gravitational vector is the negative of the K vector.
	 */
	#ifdef ACC_WEIGHT
	// Take weighted average.
	#ifdef ACC_SELF_WEIGHT
	for (i=0; i<3; i++) {
		v_acc[i] = ACC_SELF_WEIGHT * v_acc[i] + (1-ACC_SELF_WEIGHT) * v_acc_last[i];
		v_acc_last[i] = v_acc[i];

		// Kalman filtering?
		//v_acc_last[i] = acc.get(i);
		//kalmanUpdate(v_acc[i], accVar, v_acc_last[i], ACC_UPDATE_SIG);
		//kalmanPredict(v_acc[i], accVar, 0.0, ACC_PREDICT_SIG);
	}
	#endif // ACC_SELF_WEIGHT
	acc_scale = v_norm(v_acc);

	/**
	 * Reduce accelerometer weight if the magnitude of the measured
	 * acceleration is significantly greater than or less than 1 g.
	 *
	 * TODO: Magnitude of acceleration should be reported over telemetry so
	 * the "cutoff" value (the constant before the ABS() below) for
	 * disregaring acceleration input can be more accurately determined.
	 */
	#ifdef ACC_SCALE_WEIGHT
	acc_scale = (1.0 - MIN(1.0, ACC_SCALE_WEIGHT * ABS(acc_scale - 1.0)));
	acc_weight = ACC_WEIGHT * acc_scale;
	#else
	acc_weight = ACC_WEIGHT;

	// Ignore accelerometer if it measures anything 0.5g past gravity.
	if (acc_scale > 1.5 || acc_scale < 0.5) {
		acc_weight = 0;
	}
	#endif // ACC_SCALE_WEIGHT

	// Express K global unit vector in BODY frame as k_gb for use in drift
	// correction (we need K to be described in the BODY frame because
	// gravity is measured by the accelerometer in the BODY frame).
	// Technically we could just create a transpose of dcm_gyro, but since
	// we don't (yet) have a magnetometer, we don't need the first two rows
	// of the transpose. This saves a few clock cycles.
	for (i=0; i<3; i++) {
		k_gb[i] = dcm_gyro[i][2];
	}

	// Calculate gyro drift correction rotation vector w_a, which will be
	// used later to bring KB closer to the gravity vector (i.e., the
	// negative of the K vector). Although we do not explicitly negate the
	// gravity vector, the cross product below produces a rotation vector
	// that we can later add to the angular displacement vector to correct
	// for gyro drift in the X and Y axes. Note we negate w_a because our
	// acceleration vector is actually the negative of our gravity vector.
	v_crossp(k_gb, v_acc, w_a);
	v_scale(w_a, -1, w_a);
	#endif // ACC_WEIGHT

	// ========================================================================
	// Magnetometer
	//     Frame of reference: BODY
	//     Units: N/A
	//     Purpose: Measure the magnetic north vector v_mag with components
	//              codirectional with the body's i, j, and k vectors.
	// ========================================================================
	#ifdef MAG_WEIGHT
	//if (loopCount % COMM_LOOP_INTERVAL == 0) {
	//	sp("M(");
	//	sp(v_mag[0]); sp(", ");
	//	sp(v_mag[1]); sp(", ");
	//	sp(v_mag[2]);
	//	spln(")");
	//}

	// Express J global unit vectory in BODY frame as j_gb.
	for (i=0; i<3; i++) {
		j_gb[i] = dcm_gyro[i][1];
	}

	// Calculate yaw drift correction vector w_m.
	v_crossp(j_gb, v_mag, w_m);
	#endif // MAG_WEIGHT

	// ========================================================================
	// Gyroscope
	//     Frame of reference: BODY
	//     Units: rad/s
	//     Purpose: Measure the rotation rate of the body about the body's i,
	//              j, and k axes.
	// ========================================================================
	// Scale v_gyr by elapsed time (in seconds) to get angle w*dt in
	// radians, then compute weighted average with the accelerometer and
	// magnetometer correction vectors to obtain final w*dt.
	static float numerator, denominator;
	for (i=0; i<3; i++) {
		numerator   = v_gyr[i] * dt;
		denominator = 1.0;

		#ifdef ACC_WEIGHT
		numerator   += acc_weight * w_a[i];
		denominator += acc_weight;
		#endif // ACC_WEIGHT

		#ifdef MAG_WEIGHT
		numerator   += MAG_WEIGHT * w_m[i];
		denominator += MAG_WEIGHT;
		#endif // MAG_WEIGHT

		w_dt[i] = numerator / denominator;
	}

	// ========================================================================
	// Direction Cosine Matrix
	//     Frame of reference: GLOBAL
	//     Units: None (unit vectors)
	//     Purpose: Calculate the components of the body's i, j, and k unit
	//              vectors in the global frame of reference.
	// ========================================================================
	// Skew the rotation vector and sum appropriate components by combining the
	// skew symmetric matrix with the identity matrix. The math can be
	// summarized as follows:
	//
	// All of this is calculated in the BODY frame. If w is the angular
	// velocity vector, let w_dt (w*dt) be the angular displacement vector of
	// the DCM over a time interval dt. Let w_dt_i, w_dt_j, and w_dt_k be the
	// components of w_dt codirectional with the i, j, and k unit vectors,
	// respectively. Also, let dr be the linear displacement vector of the DCM
	// and dr_i, dr_j, and dr_k once again be the i, j, and k components,
	// respectively.
	//
	// In very small dt, certain vectors approach orthogonality, so we can
	// assume that (draw this out for yourself!):
	//
	//     dr_x = <    0,  dw_k, -dw_j>,
	//     dr_y = <-dw_k,     0,  dw_i>, and
	//     dr_z = < dw_j, -dw_i,     0>,
	//
	// which can be expressed as the rotation matrix:
	//
	//          [     0  dw_k -dw_j ]
	//     dr = [ -dw_k     0  dw_i ]
	//          [  dw_j -dw_i     0 ].
	//
	// This can then be multiplied by the current DCM and added to the current
	// DCM to update the DCM. To minimize the number of calculations performed
	// by the processor, however, we can combine the last two steps by
	// combining dr with the identity matrix to produce:
	//
	//              [     1  dw_k -dw_j ]
	//     dr + I = [ -dw_k     1  dw_i ]
	//              [  dw_j -dw_i     1 ],
	//
	// which we multiply with the current DCM to produce the updated DCM
	// directly.
	//
	// It may be helpful to read the Wikipedia pages on cross products and
	// rotation representation.
	// ========================================================================
	dcm_d[0][0] =        1;
	dcm_d[0][1] =  w_dt[2];
	dcm_d[0][2] = -w_dt[1];
	dcm_d[1][0] = -w_dt[2];
	dcm_d[1][1] =        1;
	dcm_d[1][2] =  w_dt[0];
	dcm_d[2][0] =  w_dt[1];
	dcm_d[2][1] = -w_dt[0];
	dcm_d[2][2] =        1;

	// Multiply the current DCM with the change in DCM and update.
	m_product(dcm_d, dcm_gyro, dcm_gyro);
	orthonormalize(dcm_gyro);

	#ifdef ACC_WEIGHT
	dcm_trim[0][0] =              1;
	dcm_trim[0][1] =              0;
	dcm_trim[0][2] = -trim_angle[1];
	dcm_trim[1][0] =              0;
	dcm_trim[1][1] =              1;
	dcm_trim[1][2] =  trim_angle[0];
	dcm_trim[2][0] =  trim_angle[1];
	dcm_trim[2][1] = -trim_angle[0];
	dcm_trim[2][2] =              1;

	// Rotate dcm_gyro with dcm_trim.
	m_product(dcm_trim, dcm_gyro, dcm_out);
	//orthonormalize(dcm_out);   // TODO: This shouldn't be necessary.
	#else
	static uint8_t j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			dcm_out[i][j] = dcm_gyro[i][j];
		}
	}
	#endif // ACC_WEIGHT
}

void debug_ahrs(uint8_t *buffer)
{
	(void)buffer;
}

