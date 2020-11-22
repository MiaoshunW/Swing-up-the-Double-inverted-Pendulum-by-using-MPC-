//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef model_h
#define model_h
#include "../base/defines.h"
#include <string>

#include <GL/gl.h>
#include "../base/vektor.h"
#include "../base/point.h"
#include "../base/color4.h"
#include "texture.h"
#include <vector>


#define GLM_NONE     (0)  /* render with only vertices */
#define GLM_FLAT     (1 << 0)  /* render with facet normals */
#define GLM_SMOOTH   (1 << 1)  /* render with vertex normals */
#define GLM_TEXTURE  (1 << 2)  /* render with texture coords */
#define GLM_COLOR    (1 << 3)  /* render with colors */
#define GLM_MATERIAL (1 << 4)  /* render with materials */


/** @ingroup glbase
 *  @brief Interface for *.obj 3d-files (wavefront).
 *
 *  Based on
 *
 *  GLM library.  Wavefront .obj file format reader.
 *
 *  Written by Nate Robins, 1997.
 *  email: ndr@pobox.com
 *  www: http://www.pobox.com/~ndr
 */
class Model {
public:
	Model();
	~Model();

	bool Load(const std::string &path, const std::string &s, double scale, double normal);
	void Draw(GLuint);
	void Draw(GLuint, int);

	bool isLoaded() {return vertices.size()!=0;}

size_t countGroups() {return groups.size();}

private:
	/** Material: Structure that defines a material in a model.
	 */
	struct Material {
		Material();
~Material();
		std::string name;    /* name of material */
		color4 diffuse;   /* diffuse component */
		color4 ambient;   /* ambient component */
		color4 specular;   /* specular component */
		color4 emmissive;   /* emmissive component */
		GLfloat shininess;   /* specular exponent */

		Texture *texture;
	};

	/** Triangle: Structure that defines a triangle in a model.
	 */
	struct Triangle {

		Vektor<unsigned int> v;   /* array of triangle vertex indices */
		Vektor<unsigned int> n;   /* array of triangle normal indices */
		Vektor<unsigned int> t;   /* array of triangle texcoord indices*/
		GLuint findex;   /* index of triangle facet normal */
	};

	/** GLMgroup: Structure that defines a group in a model.
	*/
	struct Group {

		Group(): material(0) {}
		~Group();

		/** name of this group */
		std::string name;

		/** array of triangle indices */
		std::vector<unsigned int> triangles;

		/** index to material for group */
		GLuint material;
	};




	/** name of the .obj file */
	std::string pathname;

	/** name of the material library */
	std::string mtllibname;

	/** array of vertices  */
	std::vector<Vektor<float> > vertices;

	/** array of normals  */
	std::vector<Vektor<float> > normals;

	/** array of facetnorms */
	std::vector<Vektor<float> > facetnorms;

	/** array of materials */
	std::vector<Material*> materials;

	/** array of groups */
	std::vector<Group*> groups;

	/** array of texture coordinates */
	std::vector<Point<float> > texcoords;

	/** array of triangles */
	std::vector<Triangle> triangles;


	/** "unitize" a model by translating it to the origin and
	 * scaling it to fit in a unit cube around the origin.  Returns the
	 * scalefactor used.
	 *
	 */
	GLfloat glmUnitize();

	/** Calculates the dimensions (width, height, depth) of
	 * a model.
	 *
	 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
	 */
	GLvoid glmDimensions(GLfloat* dimensions);

	/** Scales a model by a given amount.
	 *
	 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
	 */
	GLvoid glmScale(GLfloat scale);

	/** Reverse the polygon winding for all polygons in
	 * this model.  Default winding is counter-clockwise.  Also changes
	 * the direction of the normals.
	 *
	 */
	GLvoid glmReverseWinding();

	/** Generates facet normals for a model (by taking the
	 * cross product of the two vectors derived from the sides of each
	 * triangle).  Assumes a counter-clockwise winding.
	 *
	 */
	GLvoid glmFacetNormals();

	/** Generates smooth vertex normals for a model.
	 * First builds a list of all the triangles each vertex is in.  Then
	 * loops through each vertex in the the list averaging all the facet
	 * normals of the triangles each vertex is in.  Finally, sets the
	 * normal index in the triangle for the vertex to the generated smooth
	 * normal.  If the dot product of a facet normal and the facet normal
	 * associated with the first triangle in the list of triangles the
	 * current vertex is in is greater than the cosine of the angle
	 * parameter to the function, that facet normal is not added into the
	 * average normal calculation and the corresponding vertex is given
	 * the facet normal.  This tends to preserve hard edges.  The angle to
	 * use depends on the model, but 90 degrees is usually a good start.
	 *
	 * angle - maximum angle (in degrees) to smooth across
	 */
	int glmVertexNormals(GLfloat angle);

	/** Reads a model description from a Wavefront .OBJ file.
	 *
	 * filename - name of the file containing the Wavefront .OBJ format data.
	 */
	void glmReadOBJ(const std::string &filename);

	/** Renders the model to the current OpenGL context using the
	 * mode specified.
	 *
	 * mode     - a bitwise OR of values describing what is to be rendered.
	 *            GLM_NONE    -  render with only vertices
	 *            GLM_FLAT    -  render with facet normals
	 *            GLM_SMOOTH  -  render with vertex normals
	 *            GLM_COLOR   -  render with colors
	 *            GLM_TEXTURE -  render with texture coords
	 *            GLM_FLAT and GLM_SMOOTH should not both be specified.
	 */
	GLvoid glmDraw(GLuint mode);
	GLvoid glmDraw(GLuint mode, int i);

	/** Generates and returns a display list for the model using
	 * the mode specified.
	 *
	 * mode     - a bitwise OR of values describing what is to be rendered.
	 *            GLM_NONE    -  render with only vertices
	 *            GLM_FLAT    -  render with facet normals
	 *            GLM_SMOOTH  -  render with vertex normals
	 *            GLM_TEXTURE -  render with texture coords
	 *            GLM_FLAT and GLM_SMOOTH should not both be specified.
	 */
	//GLuint glmList(GLuint mode);

	/** eliminate (weld) vectors that are within an epsilon of
	 * each other.
	 *
	 * epsilon    - maximum difference between vertices
	 *              ( 0.00001 is a good start for a unitized model)
	 *
	 */
	//GLvoid glmWeld(GLfloat epsilon);


	/** returns the maximum of two floats */
	GLfloat _glmMax(GLfloat a, GLfloat b);

	/** returns the absolute value of a float */
	GLfloat _glmAbs(GLfloat f);


	/** compares two vectors and returns GL_TRUE if they are
	 * equal (within a certain threshold) or GL_FALSE if not. An epsilon
	 * that works fairly well is 0.000001.
	 *
	 * u - array of 3 GLfloats (GLfloat u[3])
	 * v - array of 3 GLfloats (GLfloat v[3])
	 */
	//GLboolean _glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon);

	/** eliminate (weld) vectors that are within an
	 * epsilon of each other.
	 *
	 * vectors    - array of GLfloat[3]'s to be welded
	 * numvectors - number of GLfloat[3]'s in vectors
	 * epsilon    - maximum difference between vectors
	 *
	 */
	//GLfloat* _glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon);

	/** Find a group in the model
	 */
	Group* _glmFindGroup(const std::string &name);

	/** Add a group to the model
	 */
	Group* _glmAddGroup(const std::string &name);

	/** Find a material in the model
	 */
	GLuint _glmFindMaterial(const std::string &name);

	/** read a wavefront material library file
	 *
	 * name  - name of the material library
	 */
	GLvoid _glmReadMTL(const std::string &name);


	/** Wavefront OBJ file that gets all the data.
	 *
	 * file  - (fopen'd) file descriptor
	 */
	GLvoid _glmPass(FILE* file);

};


#endif

