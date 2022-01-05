using UnityEngine;
using System.Collections;

public class PBD_model: MonoBehaviour {

	float 		t = 0.0333f;		// time interval
	float		damping = 0.99f;	// velocity damping
	int[] 		E;					// edge list
	float[] 	L;					// rest length of edges
	Vector3[] 	V;					// velocities of vertices
	float       g = 9.8f;			// gravitational acceleration


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n = 21;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
		for (int j = 0; j < n; j++)
		for (int i = 0; i < n; i++)
		{
			X[j*n+i]  = new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i] = new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t = 0;
		for(int j = 0; j < n-1; j++)
		for(int i = 0; i < n-1; i++)	
		{
			T[t*6+0] = j * n+i;
			T[t*6+1] = j * n + i + 1;
			T[t*6+2] = (j + 1) * n + i + 1;
			T[t*6+3] = j * n + i;
			T[t*6+4] = (j + 1) * n + i + 1;
			T[t*6+5] = (j + 1) * n + i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i = 0; i < T.Length; i += 3) 
		{
			_E[i*2+0] = T[i+0];
			_E[i*2+1] = T[i+1];
			_E[i*2+2] = T[i+1];
			_E[i*2+3] = T[i+2];
			_E[i*2+4] = T[i+2];
			_E[i*2+5] = T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
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
				E[e*2 + 0] = _E [i + 0];
				E[e*2 + 1] = _E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e] = (X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
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

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		float a = 0.2f;					// alpha of Jacobi solver

		// ===================== Position based dynamics =====================
		int n = X.Length;
		Vector3[] sum_x = new Vector3[n];	// temp xi_new (or xj_new)
		int[] sum_n = new int[n];			// temp ni (or nj)
		// initializations
		for (int i = 0; i < n; i++)
		{
			sum_x[i] = Vector3.zero;
			sum_n[i] = 0;
		}
		// get all projections
		for (int i = 0; i < E.Length; i += 2)
		{
			int vtx0 = E[i + 0];
			int vtx1 = E[i + 1];
			float Le = L[i / 2];
			sum_x[vtx0] += (X[vtx0] + X[vtx1] + Le * (X[vtx0] - X[vtx1]) / (X[vtx0] - X[vtx1]).magnitude) / 2;
			sum_x[vtx1] += (X[vtx0] + X[vtx1] - Le * (X[vtx0] - X[vtx1]) / (X[vtx0] - X[vtx1]).magnitude) / 2;
			sum_n[vtx0]++;
			sum_n[vtx1]++;
		}
		// update velocities and positions by averaging the projections above
		for (int i = 0; i < n; i++)
		{
			if (i == 0 || i == 20)	continue;	// skip the fixed points
			V[i] += ((a * X[i] + sum_x[i]) / (a + sum_n[i]) - X[i]) / t;
			X[i] = (a * X[i] + sum_x[i]) / (a + sum_n[i]);
		}
		
		mesh.vertices = X;
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

	// Update is called once per frame
	// 	1. update v and x by gravity and damping;
	//	2. apply strain limiting (PBD);
	// 	3. collision detection and response;
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20)	continue;	// skip the fixed points
			//Initial Setup
			V[i][1] -= g * t / 2;
			V[i] *= damping;
			X[i] += (V[i] * t);
		}
		mesh.vertices = X;

		for(int l = 0; l < 32; l++) 	// larger l gives larger stiffness, and vice versa.
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}


}

