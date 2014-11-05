#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

Matrix4f bezier_matrix (1,-3, 3,-1,
						0, 3,-6, 3,
						0, 0, 3,-3,
						0, 0, 0, 1);

Matrix4f spline_matrix (1,-3, 3,-1,
					    4, 0,-6, 3,
					    1, 3, 3,-3,
						0, 0, 0, 1);
						
Matrix4f Bspline_matrix = spline_matrix*(1.0f/6.0f);

Matrix4f bezier_matrix_inverse = bezier_matrix.inverse();


namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}
   

Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cout << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }
	
    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.

    // Make sure that this function computes all the appropriate
    // Vector3fs for each CurvePoint: V,T,N,B.
    // [NBT] should be unit and orthogonal.

    // Also note that you may assume that all Bezier curves that you
    // receive have G1 continuity.  Otherwise, the TNB will not be
    // be defined at points where this does not hold.
	
	float t;
	Curve R(steps);
	
	//arbitrary vector
	Vector3f B0 = Vector3f::FORWARD;

	for(int i = 0; i < steps; i++){
	
		t = (1.0f/steps)*i; 
		Matrix4f control_pt_matrix(Vector4f(P[0],0),Vector4f(P[1],0),Vector4f(P[2],0),Vector4f(P[3],0));
		
		R[i].V = (control_pt_matrix*bezier_matrix*Vector4f(1,t,t*t,t*t*t)).xyz();
		R[i].T = (control_pt_matrix*bezier_matrix*Vector4f(0,1,2*t,3*t*t)).normalized().xyz();
		
		if(i == 0) {
			if (Vector3f::cross(R[i].T, B0) == Vector3f::ZERO) B0 = Vector3f::UP;
			R[i].N = Vector3f::cross(B0,R[i].T).normalized();
		}
		else{ 
			R[i].N = Vector3f::cross(R[i-1].B,R[i].T).normalized();   
		}

		R[i].B = Vector3f::cross(R[i].T,R[i].N).normalized();
	}
		
    return R;
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{

	if( P.size() < 4){
        cout << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.

	
	/*  Imagine 8 control points.  This is how their sent to Bezier function
					
					0 1 2 3 
					1 2 3 4
					2 3 4 5
					3 4 5 6 
					4 5 6 7
	*/

	vector<Vector3f> bezier_control_points;
	Curve R;
	Curve ret;

	for(int i = 0; i < P.size()-3; i++){
		bezier_control_points.clear();
		ret.clear();

		Matrix4f control_pt_matrix(Vector4f(P[i],0),Vector4f(P[i+1],0),Vector4f(P[i+2],0),Vector4f(P[i+3],0));
		Matrix4f spline_to_bezier = control_pt_matrix*Bspline_matrix*bezier_matrix_inverse;
		
		bezier_control_points.push_back(spline_to_bezier.getCol(0).xyz());
		bezier_control_points.push_back(spline_to_bezier.getCol(1).xyz());
		bezier_control_points.push_back(spline_to_bezier.getCol(2).xyz());
		bezier_control_points.push_back(spline_to_bezier.getCol(3).xyz());
		
		ret = evalBezier(bezier_control_points,steps);
	
		
		//accumlate the curve
		R.insert(R.end(), ret.begin(), ret.end());
	}
	
    return R;
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

