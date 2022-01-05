using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
	float 		t 		 = 0.0333f;
	float 		mass	 = 1;
	float		damping	 = 0.99f;
	float 		rho		 = 0.5f;
	float 		spring_k = 8000;
	float 		g        = 9.8f;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] triangles	= new int[(n-1)*(n-1)*6];
		for(int j = 0; j < n; j++)
		for(int i = 0; i < n; i++)
		{
			X[j*n+i]  = new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i] = new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j = 0; j < n-1; j++)
		for(int i = 0; i < n-1; i++)	
		{
			triangles[t*6+0] = j * n + i;
			triangles[t*6+1] = j * n + i + 1;
			triangles[t*6+2] = (j + 1) * n + i + 1;
			triangles[t*6+3] = j * n + i;
			triangles[t*6+4] = (j + 1) * n + i + 1;
			triangles[t*6+5] = (j + 1) * n + i;
			t++;
		}
		mesh.vertices = X;
		mesh.triangles = triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();


		//Construct the original E
		int[] _E = new int[triangles.Length*2];
		for (int i = 0; i < triangles.Length; i += 3) 
		{
			_E[i*2+0] = triangles[i+0];
			_E[i*2+1] = triangles[i+1];
			_E[i*2+2] = triangles[i+1];
			_E[i*2+3] = triangles[i+2];
			_E[i*2+4] = triangles[i+2];
			_E[i*2+5] = triangles[i+0];
		}
		//Reorder the original edge list
		for (int i = 0; i < _E.Length; i += 2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i = 0; i < _E.Length; i += 2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
					e_number++;

		E = new int[e_number * 2];
		for (int i = 0, e = 0; i < _E.Length; i += 2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0] = _E [i + 0];
				E[e*2+1] = _E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e = 0; e < E.Length/2; e++) 
		{
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e] = (X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i = 0; i < V.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
    }

    void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if (l < r)
		{
			j = Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		GameObject sphere = GameObject.Find("Sphere");
		Vector3 c = sphere.transform.position;		// center position of the sphere
		float r = 2.7f;								// radius of the sphere
		
		//For every vertex, detect collision and apply impulse if needed.
		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20)	continue;	// skip the fixed points
			Vector3 d2c = X[i] - c;
			if (d2c.magnitude < r)		// consider as collided if a vertex get inside the sphere
			{
				V[i] += (c + (X[i] - c) * r / (X[i] - c).magnitude - X[i]) / t;
				X[i] = c + (X[i] - c) * r / (X[i] - c).magnitude;
			}
		}

		mesh.vertices = X;
	}

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		//Momentum and Gravity.
		for (int i = 0; i < X.Length; i++)
		{
			G[i] = mass * (X[i] - X_hat[i]) / (t * t);
			G[i][1] += mass * g;
		}
		
		//Spring Force.
		for (int idx = 0; idx < E.Length; idx += 2)
		{
			int i = E[idx + 0];
			int j = E[idx + 1];
			G[i] += spring_k * (1 - L[idx/2] / (X[i] - X[j]).magnitude) * (X[i] - X[j]);
			G[j] -= spring_k * (1 - L[idx/2] / (X[i] - X[j]).magnitude) * (X[i] - X[j]);
		}
	}

    // Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_dX 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];
		Vector3 dXi;
		float w 			= 0;

		//Initial Setup.
		for (int i = 0; i < X.Length; i++)
		{
			last_dX[i] = Vector3.zero;

			if ( i == 0 || i == 20) continue;
			V[i] *= damping;
			X_hat[i] = X[i] + V[i] * t;
			X[i] = X_hat[i];
		}

		for(int k = 0; k < 32; k++)
		{
			Get_Gradient(X, X_hat, t, G);
			
			// omega for Chebyshev acceleration
			if ( k == 0 ) w = 1;
			else if ( k == 1 ) w = 2 / (2 - rho * rho);
			else w = 4 / (4 - rho * rho * w);

			// Update X by gradient.
			for (int i = 0; i < X.Length; i++)
			{
				if ( i == 0 || i == 20) continue;
				dXi = G[i] / (mass / (t * t) + 4 * spring_k);	// current delta x
				X[i] -= (w * dXi + (1 - w) * last_dX[i]);		// update X with Chebyshev acceleration
				last_dX[i] = dXi;
			}	
		}

		// Finishing.
		for (int i = 0; i < X.Length; i++)
		{
			if ( i == 0 || i == 20) continue;
			V[i] += (X[i] - X_hat[i]) / t;
		}
		mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}
