using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	public bool launched = false;
	Vector3[] X;							// vertices' positions
	Vector3[] Q;
	Vector3[] V;							// vertices' velocities
	Matrix4x4 QQt = Matrix4x4.zero;
	float g = 9.8f;							// gravitational acceleration
	float uN = 0.5f;
	float uT = 0.5f;
	float linear_decay = 0.999f;      		// for velocity decay


    // Start is called before the first frame update
    void Start()
    {
    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c = Vector3.zero;
        for (int i = 0; i < Q.Length; i++)
        	c += Q[i];
        c /= Q.Length;
        for(int i = 0; i < Q.Length; i++)
        	Q[i] -= c;

        //Get QQ^t ready.
		for (int i = 0; i < Q.Length; i++)
		{
			QQt[0, 0] += Q[i][0] * Q[i][0];
			QQt[0, 1] += Q[i][0] * Q[i][1];
			QQt[0, 2] += Q[i][0] * Q[i][2];
			QQt[1, 0] += Q[i][1] * Q[i][0];
			QQt[1, 1] += Q[i][1] * Q[i][1];
			QQt[1, 2] += Q[i][1] * Q[i][2];
			QQt[2, 0] += Q[i][2] * Q[i][0];
			QQt[2, 1] += Q[i][2] * Q[i][1];
			QQt[2, 2] += Q[i][2] * Q[i][2];
		} 
	 	QQt[3, 3]=1;

		for(int i = 0; i < X.Length; i++)
			V[i][0] = 5.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position = Vector3.zero;
		transform.rotation = Quaternion.identity;
   	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	Matrix4x4 Get_Outer_Product(Vector3 a, Vector3 b) 
	{
		Matrix4x4 M = Matrix4x4.zero;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				M[i, j] = a[i] * b[j];
			}
		}
		M[3, 3] = 1;
		return M;
	}

	Matrix4x4 Get_Matrix_Sum(Matrix4x4 B, Matrix4x4 C)
    {
        // Matrix4x4 addition: A = B + C
        Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) A.SetColumn(i, B.GetColumn(i) + C.GetColumn(i));
        return A;
    }

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i = 0; i < Q.Length; i++)
		{
			Vector3 x = (Vector3)(R * Q[i]) + c;

			V[i] += (x - X[i]) * inv_dt;
			X[i] = x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = X;
   	}

	void Collision(float inv_dt, Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);		// rotation matrix
		Vector3 c = transform.position;
		int count = 0;

		for (int i = 0; i < X.Length; i++)
		{
			Vector3 Rri = R.MultiplyVector(X[i]);
			Vector3 xi = c + Rri;
			Vector3 dis = xi - P;
			float phi_xi = Vector3.Dot(dis, N);

			if (phi_xi < 0)
			{
				X[i] -= phi_xi * N;
				if (Vector3.Dot(V[i], N) < 0)
				{
					Vector3 vN = Vector3.Dot(V[i], N) * N;
					Vector3 vT = V[i] - vN;
					float a = Mathf.Max(1 - uT * (1 + uN) * vN.magnitude / vT.magnitude, 0);
					vN *= (-uN);
					vT *= a;
					V[i] = vN + vT;
					count++;
				}
			}
		}
		if (count > 0) uN *= 0.5f;		// restitution decay
	}

    // Update is called once per frame
    void Update()
    {
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			transform.rotation = Quaternion.identity;
			QQt = Matrix4x4.zero;
			Start();
			uN = 0.5f;
			launched = false;
		}
		if(Input.GetKey("l"))
		{
			launched = true;
		}
  		float dt = 0.015f;

		if (launched) 
		{
			//Step 1: run a simple particle system.
			for (int i = 0; i < V.Length; i++)
			{
				V[i][1] -= g * dt;
				V[i] *= linear_decay;
				X[i] += V[i] * dt;
			}

			//Step 2: Perform simple particle collision.
			Collision(1/dt, new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision(1/dt, new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Step 3: Use shape matching to get new translation c and 
			// new rotation R. Update the mesh by c and R.
			//Shape Matching (translation)
			Vector3 c = new Vector3(0, 0, 0);
			for (int i = 0; i < X.Length; i++) c += X[i];
			c /= X.Length;
			
			//Shape Matching (rotation)
			Matrix4x4 A0 = Matrix4x4.zero;
			for (int i = 0; i < X.Length; i++)
			{
				A0 = Get_Matrix_Sum(A0, Get_Outer_Product(X[i] - c, Q[i]));
			}
			Matrix4x4 A = A0 * QQt.inverse;
			Matrix4x4 R = Get_Rotation(A);

			Update_Mesh(c, R, 1/dt);
		}
    }
}
