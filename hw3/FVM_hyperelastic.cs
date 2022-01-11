using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM_hyperelastic : MonoBehaviour
{
	float dt 			= 0.002f;
    float mass 			= 1;
	float stiffness_0	= 15000.0f;
    float stiffness_1 	= 7500.0f;
    float damp			= 0.999f;
	float g    			= 9.8f;
	float uT			= 0.5f;
	float uN 			= 0.5f;
	Vector3 floorN		= new Vector3(0, 1, 0);
	Vector3 floorP		= new Vector3(0, -3, 0);

	int[] 		Tet;
	int tet_number;			// The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int[][] neighbors;		// neighbors of vertices
	int number;				// The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number = int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet = 0; tet < tet_number; tet++)
    		{
				Tet[tet*4+0] = int.Parse(Strings[tet*5+4]) - 1;
				Tet[tet*4+1] = int.Parse(Strings[tet*5+5]) - 1;
				Tet[tet*4+2] = int.Parse(Strings[tet*5+6]) - 1;
				Tet[tet*4+3] = int.Parse(Strings[tet*5+7]) - 1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i = 0; i < number; i++)
       		{
       			X[i].x = float.Parse(Strings[i*5+5]) * 0.4f;
       			X[i].y = float.Parse(Strings[i*5+6]) * 0.4f;
       			X[i].z = float.Parse(Strings[i*5+7]) * 0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i = 0; i < number; i++)		center += X[i];
	    	center = center / number;
	    	for(int i = 0; i < number; i++)
	    	{
	    		X[i] -= center;
	    		float temp = X[i].y;
	    		X[i].y = X[i].z;
	    		X[i].z = temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
        	vertices[vertex_number++] = X[Tet[tet*4+1]];
 
         	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
 
         	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+1]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
 
         	vertices[vertex_number++] = X[Tet[tet*4+1]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
        } 
 
        int[] triangles = new int[tet_number*12];
        for(int t = 0; t < tet_number * 4; t++)
        {
        	triangles[t*3+0] = t*3+0;
        	triangles[t*3+1] = t*3+1;
        	triangles[t*3+2] = t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int tet = 0; tet < tet_number; tet++)
		{
			inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;
		}

		int[,] links = new int[number, number];
		for (int tet = 0; tet < tet_number; tet++)
		{
			links[Tet[tet*4+0], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+0], Tet[tet*4+2]] = 1;
			links[Tet[tet*4+0], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+2]] = 1;
			links[Tet[tet*4+1], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+2], Tet[tet*4+3]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+0]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+1]] = 1;
			links[Tet[tet*4+3], Tet[tet*4+2]] = 1;
		}

		neighbors = new int[number][];
		for (int i = 0; i < number; i++)
		{
			for (int j = i + 1; j < number; j++)
			{
				if (links[i, j] == 1)
				{
					V_num[i] += 1;
					V_num[j] += 1;
				}
			}
			neighbors[i] = new int[V_num[i]];
		}

		int[] counts = new int[number];
		for (int i = 0; i < number - 1; i++)
		{
			for (int j = i + 1; j < number; j++)
			{
				if (links[i, j] == 1)
				{
					neighbors[i][counts[i]] = j;
					neighbors[j][counts[j]] = i;
					counts[i]++;
					counts[j]++;
				}	
			}
		}

    }

	Matrix4x4 Dot_Number(float k, Matrix4x4 B)
    {
        // matrix number multiplication: A = k * B
        Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) A.SetColumn(i, k * B.GetColumn(i));
        return A;
    }

	float Trace(Matrix4x4 A)
	{
		return A[0, 0] + A[1, 1] + A[2, 2];
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

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret = Matrix4x4.zero;
    	// Build edge matrix here
		Vector3 X10 = X[Tet[tet*4 + 1]] - X[Tet[tet*4 + 0]];
		Vector3 X20 = X[Tet[tet*4 + 2]] - X[Tet[tet*4 + 0]];
		Vector3 X30 = X[Tet[tet*4 + 3]] - X[Tet[tet*4 + 0]];

		ret.SetColumn(0, X10);
		ret.SetColumn(1, X20);
		ret.SetColumn(2, X30);
		ret[3, 3] = 1;

		return ret;
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i = 0; i < number; i++)
    			V[i].y += 0.2f;
    	}

    	for(int i = 0; i < number; i++)
    	{
    		// Add gravity to Force.
			Force[i] = new Vector3(0, - mass * g, 0);
    	}

    	for(int tet = 0; tet < tet_number; tet++)
    	{
    		// Deformation Gradient
			Matrix4x4 curEdgeMat = Build_Edge_Matrix(tet);
			Matrix4x4 F = curEdgeMat * inv_Dm[tet];
			F[3, 3] = 1;

    		// SVD
			Matrix4x4 U = Matrix4x4.zero;
            Matrix4x4 S = Matrix4x4.zero;
            Matrix4x4 VT = Matrix4x4.zero;
            SVD SVD = new SVD();
            SVD.svd(F, ref U, ref S, ref VT);
            VT = VT.transpose;

    		// StVK model & Neo-Hookean model
            Vector3 lambda = new Vector3(S[0, 0], S[1, 1], S[2, 2]);    // lambda0, lambda1, lambda2
            Vector3 lambda_2 = Vector3.Scale(lambda, lambda);           // lambda0^2, lambda1^2, lambda2^2
            Vector3 Is = new Vector3(
                lambda[0] + lambda[1] + lambda[2], 
                lambda_2[0] + lambda_2[1] + lambda_2[2], 
                lambda[0] * lambda[1] * lambda[2]);                     // I, II, III
            Matrix4x4 D = Matrix4x4.identity;
            // StVK
            // for (int i = 0; i < 3; i++)
            // {
            //     D[i, i] = 2 * stiffness_0 * S[i, i] * (Is[0] - 3) + stiffness_1 * S[i, i] * (S[i, i] * S[i, i] - 1);
            // }

            // Neo-Hookean
            for (int i = 0; i < 3; i++)
            {
                D[i, i] = stiffness_0 * (- 2 * Mathf.Pow(Is[2], -1.0f/3) * Is[0] / S[i, i] / 3 + 2 * Mathf.Pow(Is[2], 1.0f/3) * S[i, i]) 
                        + stiffness_1 * (- Mathf.Pow(Is[2], -1.0f/2) / S[i, i]);
            }

            // Second PK Stress
			Matrix4x4 P = U * D * VT;

    		// Elastic Force
			Matrix4x4 f123 = Dot_Number(- 1 / (6 * inv_Dm[tet].determinant), P * inv_Dm[tet].transpose);
			f123[3, 3] = 1;
			Vector3 f1 = f123.GetColumn(0);
			Vector3 f2 = f123.GetColumn(1);
			Vector3 f3 = f123.GetColumn(2);
			Vector3 f0 = - f1 - f2 - f3;
			Force[Tet[tet*4 + 0]] += f0;
			Force[Tet[tet*4 + 1]] += f1;
			Force[Tet[tet*4 + 2]] += f2;
			Force[Tet[tet*4 + 3]] += f3;
    	}

    	for(int i = 0; i < number; i++)
    	{
    		// Update X and V here.
			V[i] += (Force[i] / mass) * dt;
			V[i] *= damp;
			
			V[i] = (V_num[i] * V[i] + V_sum[i]) / (2 * V_num[i]);

			X[i] += V[i] * dt;

    		// (Particle) collision with floor.
			Vector3 dis = X[i] - floorP;
			float phi_xi = Vector3.Dot(dis, floorN);

			if (phi_xi < 0)
			{
				X[i] -= phi_xi * floorN;
				if (Vector3.Dot(V[i], floorN) < 0)
				{
					Vector3 vN = Vector3.Dot(V[i], floorN) * floorN;
					Vector3 vT = V[i] - vN;
					float a = Mathf.Max(1 - uT * (1 + uN) * vN.magnitude / vT.magnitude, 0);
					vN *= (-uN);
					vT *= a;
					V[i] = vN + vT;
				}
			}

    	}
		for(int i = 0; i < number; i++)
		{
			V_sum[i] = Vector3.zero;
			for (int j = 0; j < neighbors[i].Length; j++)
			{
				V_sum[i] += V[neighbors[i][j]];
			}
		}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l = 0; l < 10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number = 0;
        for(int tet = 0; tet < tet_number; tet++)
        {
        	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
        	vertices[vertex_number++] = X[Tet[tet*4+1]];
        	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
        	vertices[vertex_number++] = X[Tet[tet*4+0]];
        	vertices[vertex_number++] = X[Tet[tet*4+1]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
        	vertices[vertex_number++] = X[Tet[tet*4+1]];
        	vertices[vertex_number++] = X[Tet[tet*4+2]];
        	vertices[vertex_number++] = X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals();
    }
}
