using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(5, 2, 0);	// velocity
	Vector3 w 			= new Vector3(0.5f, 0, 0);	// angular velocity
	float g             = 9.8f;					// gravitational acceleration
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision
	float uT 			= 0.5f;					// for friction
	Vector3[] vertices;

	// ================= Use this for initialization =================
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	// ================= utils =================
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		// Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Matrix4x4 Get_Matrix_Sum(Matrix4x4 B, Matrix4x4 C)
    {
        // Matrix4x4 addition: A = B + C
        Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) A.SetColumn(i, B.GetColumn(i) + C.GetColumn(i));
        return A;
    }

	Matrix4x4 Get_Negative(Matrix4x4 B)
    {
        // Negative matrix: A = - B
        Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) A.SetColumn(i, - B.GetColumn(i));
        return A;
    }

	Matrix4x4 Dot_Number(float k, Matrix4x4 B)
    {
        // matrix number multiplication: A = k * B
        Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) A.SetColumn(i, k * B.GetColumn(i));
        return A;
    }

	// ================= Main =================
	// In this function, update v and w by the impulse due to the collision with 
	// a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		int numCollisionPoint = 0;
		Vector3 avg_p = new Vector3(0 ,0, 0);		// average collision point
		Vector3 avg_v = new Vector3(0, 0, 0);		// average collision velocity
		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);		// rotation matrix

		// collision detection
		for (int i = 0; i < vertices.Length; i++) 
		{
			Vector3 Rri = R.MultiplyVector(vertices[i]);
			Vector3 xi = x + Rri;
			Vector3 dis = xi - P;
			float phi_xi = Vector3.Dot(dis, N);

			if (phi_xi < 0) 
			{
				Vector3 vi = v + Vector3.Cross(w, Rri);
				if (Vector3.Dot(vi, N) < 0)
				{
					avg_v += vi;
					avg_p += Rri;
					numCollisionPoint++;
				}
			}
		}
		if (numCollisionPoint == 0) return;		// not collide
		avg_p /= numCollisionPoint;				// get average collision point
		avg_v /= numCollisionPoint;				// get average velocity

		// update vertex velocity
		Vector3 v_Ni = Vector3.Dot(avg_v, N) * N;
		Vector3 v_Ti = avg_v - v_Ni;
		float a = Mathf.Max(1 - uT * (1 + restitution) * v_Ni.magnitude / v_Ti.magnitude, 0);
		
		v_Ni *= (-restitution);
		v_Ti *= a;
		Vector3 vi_new = v_Ni + v_Ti;

		// compute K
		Matrix4x4 avg_p_cross = Get_Cross_Matrix(avg_p);
		Matrix4x4 inertia = R * I_ref * R.transpose;
		Matrix4x4 Iinv = inertia.inverse;
		Matrix4x4 K = Get_Matrix_Sum(Dot_Number(1 / mass, Matrix4x4.identity), Get_Negative(avg_p_cross * Iinv * avg_p_cross));
		
		// get impulse
		Vector3 impulse = K.inverse.MultiplyVector(vi_new - avg_v);

		// update linear and angular velocity, and reduce restitution
		v += impulse / mass;
		w += (Iinv * avg_p_cross).MultiplyVector(impulse);
		restitution *= 0.5f;
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3(0.5f, 0, 0);
			launched = true;
		}

		if (launched) 
		{
			// Part I: Update velocities
			v [1] -= (dt * g);
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x    = transform.position;
			x = x + v * dt;
			// Update angular status --- q1 = q0 + [0 w*dt/2] x q0 = [sq0 vq0] + [sw vw] x [sq0 vq0]
			Quaternion q0 = transform.rotation;
			float sq0 = q0.w;
			Vector3 vq0 = new Vector3(q0.x, q0.y, q0.z);
			Vector3 vw = w * dt / 2;
			float sdq = - Vector3.Dot(vw, vq0);
			Vector3 vdq = sq0 * vw + Vector3.Cross(vw, vq0);
			Quaternion q1 = new Quaternion(vdq[0] + vq0[0], vdq[1] + vq0[1], vdq[2] + vq0[2], sdq + sq0);

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q1;
		}
	}
}
