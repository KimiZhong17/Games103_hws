using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int 		size 			= 100;
	float	 	rate 			= 0.005f;
	float 		gamma			= 0.002f;
	float 		damping 		= 0.996f;
	float 		bottom			= -2.0f;
	float	 	linear_decay	= 0.999f;
	float 		angular_decay	= 0.98f;
	float 		dt 				= 0.002f;
	float 		g				= -9.8f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	// bool 		tag				= true;

	GameObject 	block1;
	GameObject 	block2;

	Vector3 	block1_v 		= Vector3.zero;
	Vector3 	block1_w 		= Vector3.zero;
	Vector3 	block2_v 		= Vector3.zero;
	Vector3 	block2_w 		= Vector3.zero;

	float 		block1_mass;
	float 		block2_mass;
	
	float 		block1_buoyancy;
	float 		block2_buoyancy;
	Vector3		torque_1;
	Vector3		torque_2;
	Matrix4x4	I_ref_1;
	Matrix4x4	I_ref_2;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X = new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}

		block1 = GameObject.Find("Block");
		block2 = GameObject.Find("Cube");


		Vector3[] X1 = block1.GetComponent<MeshFilter>().mesh.vertices;
		float m = 1;
		block1_mass = 0;
		for (int i = 0; i < X1.Length; i++) 
		{
			block1_mass += m;
			float diag = m * X1[i].sqrMagnitude;
			I_ref_1[0, 0] += diag;
			I_ref_1[1, 1] += diag;
			I_ref_1[2, 2] += diag;
			I_ref_1[0, 0] -= m * X1[i][0] * X1[i][0];
			I_ref_1[0, 1] -= m * X1[i][0] * X1[i][1];
			I_ref_1[0, 2] -= m * X1[i][0] * X1[i][2];
			I_ref_1[1, 0] -= m * X1[i][1] * X1[i][0];
			I_ref_1[1, 1] -= m * X1[i][1] * X1[i][1];
			I_ref_1[1, 2] -= m * X1[i][1] * X1[i][2];
			I_ref_1[2, 0] -= m * X1[i][2] * X1[i][0];
			I_ref_1[2, 1] -= m * X1[i][2] * X1[i][1];
			I_ref_1[2, 2] -= m * X1[i][2] * X1[i][2];
		}
		I_ref_1 [3, 3] = 1;

		Vector3[] X2 = block2.GetComponent<MeshFilter>().mesh.vertices;
		block2_mass = 0;
		for (int i = 0; i < X2.Length; i++) 
		{
			block2_mass += m;
			float diag = m * X2[i].sqrMagnitude;
			I_ref_2[0, 0] += diag;
			I_ref_2[1, 1] += diag;
			I_ref_2[2, 2] += diag;
			I_ref_2[0, 0] -= m * X2[i][0] * X2[i][0];
			I_ref_2[0, 1] -= m * X2[i][0] * X2[i][1];
			I_ref_2[0, 2] -= m * X2[i][0] * X2[i][2];
			I_ref_2[1, 0] -= m * X2[i][1] * X2[i][0];
			I_ref_2[1, 1] -= m * X2[i][1] * X2[i][1];
			I_ref_2[1, 2] -= m * X2[i][1] * X2[i][2];
			I_ref_2[2, 0] -= m * X2[i][2] * X2[i][0];
			I_ref_2[2, 1] -= m * X2[i][2] * X2[i][1];
			I_ref_2[2, 2] -= m * X2[i][2] * X2[i][2];
		}
		I_ref_2 [3, 3] = 1;
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Block2WaterCoupling(GameObject block, float[,] new_h, ref float buoyancy, ref Vector3 torque, Vector3[] X)
	{
		// Block -> Water coupling
		// calculate low_h, b, and cg_mask, then solve the Poisson equation to obtain vh (virtual height)
		Collider collider = block.GetComponent<Collider>();
		Bounds bounds = collider.bounds;
		Vector3[,] Rri = new Vector3[size, size];

		int li = (int) Mathf.Max(((bounds.min.x - 1) / 0.1f) + size / 2, 0);
		int ui = (int) Mathf.Min(((bounds.max.x + 1) / 0.1f) + size / 2, size-1);
		int lj = (int) Mathf.Max(((bounds.min.z - 1) / 0.1f) + size / 2, 0);
		int uj = (int) Mathf.Min(((bounds.max.z + 1) / 0.1f) + size / 2, size-1);

		for (int i = 0; i < size; i++) 
		{
			for (int j = 0; j < size; j++)
			{
				cg_mask[i, j] = false;
				b[i, j] = 0;
				if ( i >= li && i <= ui && j >= lj & j <= uj )
				{
					Ray ray = new Ray(new Vector3(X[i*size + j].x, bottom, X[i*size + j].z), Vector3.up);
					if (collider.Raycast(ray, out RaycastHit hitInfo, 2.0f))
					{
						float depth = bottom + hitInfo.distance;
						if (depth < 0)
						{
							cg_mask[i, j] = true;
							low_h[i, j] = depth;
							b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
							Rri[i, j] = hitInfo.point - block.transform.position;
						}
					}
				}
			}
		}

		Conjugate_Gradient(cg_mask, b, vh, li, ui, lj, uj);

		for (int i = 0; i < size; i++) 
		{
			for (int j = 0; j < size; j++)
			{
				if (cg_mask[i, j]) 
				{
					buoyancy += vh[i, j];
					torque += Vector3.Cross(Rri[i, j], new Vector3(0, vh[i, j] * 0.3f, 0));
				}
			}
		}
	}

	Quaternion updateRotation(Quaternion q0, Vector3 w)
	{
		// update rotation based on angular velocity
		float sq0 = q0.w;
		Vector3 vq0 = new Vector3(q0.x, q0.y, q0.z);
		Vector3 vw = w * dt / 2;
		float sdq = - Vector3.Dot(vw, vq0);
		Vector3 vdq = sq0 * vw + Vector3.Cross(vw, vq0);
		return new Quaternion(vdq[0] + vq0[0], vdq[1] + vq0[1], vdq[2] + vq0[2], sdq + sq0);
	}

	void Water2BlockCoupling(ref GameObject block, ref Vector3 v, ref Vector3 w, float mass, float buoyancy, Vector3 torque, Matrix4x4 I_ref)
	{
		// perform Water -> Block coupling
		Vector3 x = block.transform.position;
		v[1] += (dt * g + buoyancy * dt / mass * 0.012f);
		v *= linear_decay;
		x = x + v * dt;

		Quaternion q = block.transform.rotation;
		Matrix4x4 R = Matrix4x4.Rotate(q);
        Matrix4x4 I = R * I_ref * R.transpose;
		w += I.inverse.MultiplyVector(torque) * dt;
		w *= angular_decay;

		q = updateRotation(q, w);
		
		block.transform.position = x;
		block.transform.rotation = q;
	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{	
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		block1_buoyancy = 0;
		block2_buoyancy = 0;
		torque_1 = new Vector3(0, 0, 0);
		torque_2 = new Vector3(0, 0, 0);
		
		//Step 1: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; i++){
			int lower = Mathf.Max(0, i-1);
			int upper = Mathf.Min(size-1, i+1);
			for (int j = 0; j < size; j++)
			{
				int left = Mathf.Max(0, j-1);
				int right = Mathf.Min(size-1, j+1);
				float smoothing = h[lower, j] + h[upper, j] + h[i, left] + h[i, right] - 4 * h[i, j];
				new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping + smoothing * rate;
			}
		}

		//Step 2: Block->Water coupling for two blocks respectively
		Block2WaterCoupling(block1, new_h, ref block1_buoyancy, ref torque_1, X);
		Block2WaterCoupling(block2, new_h, ref block2_buoyancy, ref torque_2, X);
	
		// Diminish vh.
		for (int i = 0; i < size; i++){
			for (int j = 0; j < size; j++)
			{
				vh[i, j] *= gamma;
			}
		}

		// Update new_h by vh.
		for (int i = 0; i < size; i++){
			int lower = Mathf.Max(0, i-1);
			int upper = Mathf.Min(size-1, i+1);
			for (int j = 0; j < size; j++)
			{
				int left = Mathf.Max(0, j-1);
				int right = Mathf.Min(size-1, j+1);
				new_h[i, j] += (vh[lower, j] + vh[upper, j] + vh[i, left] + vh[i, right] - 4 * vh[i, j]) * rate;
			}
		}

		//Step 3: old_h <- h; h <- new_h;
		for (int i = 0; i < size; i++){
			for (int j = 0; j < size; j++)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}

		//Step 4: Water->Block coupling.
		Water2BlockCoupling(ref block1, ref block1_v, ref block1_w, block1_mass, block1_buoyancy, torque_1, I_ref_1);
		Water2BlockCoupling(ref block2, ref block2_v, ref block2_w, block2_mass, block2_buoyancy, torque_2, I_ref_2);
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < X.Length; i++)
		{
			h[i / size, i % size] = X[i].y;
		}

		if (Input.GetKeyDown ("r")) 
		{
			// Add random water.
			int _i = Random.Range(0, size);
			int _j = Random.Range(0, size);
			int top_i = Mathf.Max(0, _i-1);
			int bottom_i = Mathf.Min(size-1, _i+1);
			int left_j = Mathf.Max(0, _j-1);
			int right_j = Mathf.Min(size-1, _j+1);
			float r = Random.Range(0.1f, 0.5f);
			h[_i, _j] += r;
			h[top_i, left_j] -= r / 8.0f;
			h[top_i, _j] -= r / 8.0f;
			h[top_i, right_j] -= r / 8.0f;
			h[_i, left_j] -= r / 8.0f;
			h[_i, right_j] -= r / 8.0f;
			h[bottom_i, left_j] -= r / 8.0f;
			h[bottom_i, _j] -= r / 8.0f;
			h[bottom_i, right_j] -= r / 8.0f;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		// Store h back into X.y and recalculate normal.
		for (int i = 0; i < X.Length; i++)
		{
			X[i].y = h[i / size, i % size];
		}
		mesh.vertices  = X;
		mesh.RecalculateNormals ();
		
	}
}
