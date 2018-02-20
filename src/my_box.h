#ifndef MY_BOX_H
#define MY_BOX_H

/*these structs are for collision*/
struct d_min_struct
{
	float d_min;		//final d_min
	float d_min_faces;	//d_min of all faces
	float d_min_edges;	//d_min of all edges
	float s_min[3];
	float s_min_faces[3];
	float s_min_edges[3];
	int is_initialized;
	int source;	//type of source for s_min. 0 = face, 1 = edge
	int i_face;	//index of face that made s_min
	int i_edge[2];	//index of edges that made s_min
	int i_hull[2];		//hulls that made s_min. element 0 = hull /w reference face. 1 = hull /w incident face
	int cur_check; //type of check being currently done. 0 = face, 1 = edge
};

struct face_struct
{
	float normal[3];
	int i_vertices[4]; //indices of the vertices that make up the face.
	int num_verts;	//could be 3 or 4
};

struct edge_struct
{
	float normal[3];	//unit vector that is cross product of faces 
	int i_vertices[2]; //indices of the two vertices that make up the face.
	int i_face[2];  //adjacent faces index
};

struct box_collision_struct
{
	float * positions;	//array of vec3's (use array from no_tex_model_struct)
	struct face_struct * faces;
	struct edge_struct * edges;
	int num_pos;	//# of pos vertices
	int num_faces;
	int num_edges;
};

struct box_struct
{
	float mass;
	float imomentOfInertia[9];	//inverse-moment-of-inertia in local space.
	float pos[3];
	float linearVel[3];
	float orientation[9];	//mat3
	float orientationQ[4];
	float angularMomentum[3];
	float angularVel[3];
	float newLinearVel[3];
	float newAngularMomentum[3];
	float newAngularVelQ[4];
	struct box_collision_struct hull;
};

struct contact_info_struct
{
	float point[3];		//world-space contact point
	float normal[3];	//from object B to A
	float penetration;	//negative for separated objects
};

struct contact_manifold_struct
{
	struct contact_info_struct contacts[4];
	struct box_struct * overlappedBoxes[2]; //the two boxes in contact, [0] = boxA, [1] = boxB
	int num_contacts;

};

#endif 
