#include "surf.h"
#include "extra.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0){
				
				profile[i].V.Write();
				profile[i].T.Write();
				profile[i].N.Write();
				return false;
			}
               
    
        return true;
    }
}

void generateMesh(Surface &surface, const Curve &profile, int steps, int idx,Matrix4f &m){
	
	int size = profile.size();
	Tup3u V;

	for(int j = 0; j < profile.size(); j++){ 
			
			surface.VV.push_back((m*Vector4f(profile[j].V,1)).xyz());
			m.inverse().transpose();
			surface.VN.push_back((m*Vector4f (profile[j].N,0)).xyz());

			//Build two triangles to represent solid square patch
			if(j != profile.size()-1){
				V[0] = idx*profile.size() + j;
				V[1] = V[0] + 1;
				V[2] = (V[0] + profile.size())%(profile.size()*steps);
				surface.VF.push_back(V);
				surface.VF.push_back(Tup3u (V[1],V[2]+1,V[2]));
			}
		}
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
	float t;
	Matrix4f ry;
    

    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        getchar();
		exit(0);
    }

	for(int i = 0; i < steps; i++){
		t = (2.0f*M_PI/steps)*float(i);		
		ry = Matrix4f::rotateY(t);
		generateMesh(surface,profile,steps,i,ry);
	}

    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        getchar();
		exit(0);
    }

	Matrix4f m;
    
	for(int i = 0; i < sweep.size(); i++){
		m.setCol(0,Vector4f(sweep[i].N,0));
		m.setCol(1,Vector4f(sweep[i].B,0));
		m.setCol(2,Vector4f(sweep[i].T,0));
		m.setCol(3,Vector4f(sweep[i].V,1));

		generateMesh(surface,profile,sweep.size(),i,m);
	}

    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
		glColor3f(0,1.0f,0);
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
