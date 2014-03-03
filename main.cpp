#include <iostream>
#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
#include <pthread.h>

//#include <windows.h>
#include <unistd.h>
#include <cstdlib>

#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_ttf.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

using namespace std;

typedef double pScalar;

void Sleep( int t ) {
    usleep(t*1000);
}

/*-------------------------------------------------------*/

class pVector {

public:
    pScalar x;
    pScalar y;
    pVector scale(pScalar lambda);
    pVector normalize();
    pScalar length();

    void print();
};

void pVector::print() {
    cout << "[V: X: " << x << " Y: " << y << " ]" << endl;
}

pVector add(pVector a, pVector b) {
    pVector result;

    result.x = a.x + b.x;
    result.y = a.y + b.y;

    return result;
}

pVector sub(pVector a, pVector b) {
    pVector result;

    result.x = a.x - b.x;
    result.y = a.y - b.y;

    return result;
}

pVector pVector::scale(pScalar lambda) {
    pVector result;

    result.x = x*lambda;
    result.y = y*lambda;

    return result;
}

pScalar pVector::length() {
    return sqrt(x*x + y*y);
}

pVector pVector::normalize() {
    return scale( 1.0/length() );
}

pScalar scalarProduct( pVector a, pVector b ) {
    return a.x*b.x + a.y*b.y;
}

//Cosine of angle between a and b
pScalar vCos(pVector a, pVector b) {
    return scalarProduct(a,b)/(a.length() * b.length());
}

//Unit normal vector to line defined by a and b
pVector normal( pVector a, pVector b ) {
    pVector base = sub(b,a).normalize();
    pVector result;
    result.x = -base.y;
    result.y = base.x;
    return result;
}

pVector normal( pVector b ) {
    pVector result;
    result.x = -b.y;
    result.y = b.x;
    return result;
}

pScalar crossDir( pVector a, pVector b ) {
    return ((a.x*b.y)-(a.y*b.x));
}

/*-------------------------------------------------------*/

class Transform {

public:
    pScalar a11,a12,a21,a22;
    pVector offset;

    Transform(pScalar a11, pScalar a12, pScalar a21,pScalar a22, pScalar tx, pScalar ty);
    Transform(pScalar a11, pScalar a12, pScalar a21, pScalar a22, pVector offset);
    Transform(pScalar angle, pVector offset);
    Transform();

    void print();

    pVector apply(pVector v);
    pVector applyInverse(pVector v);
    pVector applyRotation(pVector v);
    void set(pScalar angle, pVector offset);
};

void Transform::print() {
    cout << "[Transform]" << endl;
    cout << "M:" << endl;
    cout << "[" << a11 << "," << a12 << "]" << endl;
    cout << "[" << a21 << "," << a22 << "]" << endl;
    cout << "V:" << endl;
    cout << "[" << offset.x << "," << offset.y << "]" << endl;
}

pVector Transform::apply(pVector v) {
    pVector result;

    result.x = v.x*a11 + v.y*a12;
    result.y = v.x*a21 + v.y*a22;

    result = add(result, offset);

    return result;
}

pVector Transform::applyRotation(pVector v) {
    pVector result;

    result.x = v.x*a11 + v.y*a12;
    result.y = v.x*a21 + v.y*a22;

    return result;
}

pVector Transform::applyInverse(pVector v) {
    pVector result;
    pVector temp;

    temp = sub(v, offset);

    result.x = temp.x*a22 - temp.y*a12;
    result.y = -temp.x*a21 + temp.y*a11;

    return result;
}

Transform::Transform(pScalar a11, pScalar a12, pScalar a21,pScalar a22, pScalar tx, pScalar ty) {
    a11 = a11;
    a12 = a12;
    a21 = a21;
    a22 = a22;
    offset.x = tx;
    offset.y = ty;
}

Transform::Transform(pScalar a11, pScalar a12, pScalar a21, pScalar a22, pVector offsetArg) {
    a11 = a11;
    a12 = a12;
    a21 = a21;
    a22 = a22;
    offset = offsetArg;
}

Transform::Transform(pScalar angle, pVector offsetArg) {
    a11 = cos(angle);
    a12 = -sin(angle);
    a21 = sin(angle);
    a22 = cos(angle);
    offset = offsetArg;
}

Transform::Transform() {
    a11 = 1.0;
    a12 = 0.0;
    a21 = 0.0;
    a22 = 1.0;
    offset.x = 0.0;
    offset.y = 0.0;
}

void Transform::set(pScalar angle, pVector offsetArg) {
    a11 = cos(angle);
    a12 = -sin(angle);
    a21 = sin(angle);
    a22 = cos(angle);
    offset = offsetArg;
}

/*-------------------------------------------------------*/

//#define GEO_BOX    11
#define GEO_CONVEX 13

//For convex geometry points is an array of vertices
//: l1 ... ln

class Geometry {

public:
    int type;
    pScalar radius;
    vector<pVector> points;

    pVector getFace( unsigned int i ) {

        assert( i < points.size() );

        pVector res;
        if( i < (points.size() - 1) ) {
            res = sub( points[i+1], points[i] );
        } else {
            res = sub( points[0], points[i] );
        }
        return res;
    }

    pVector getNormal( unsigned int i ) {

        // cout << "[getNormal: " << (int)this << " ]" << endl;
        //cout << i << endl;

        pVector base = getFace( i ).normalize();
        pVector result;
        result.x = -base.y;
        result.y = base.x;
        return result;
    }

    pVector getFaceUnit( unsigned int i ) {
        return getFace( i ).normalize();
    }

    void updateRadius() {

        unsigned int i = 0;
        radius = 0.0;

        for( i=0; i<points.size(); i++ ) {
            if( points[i].length() > radius ) {
                radius = points[i].length();
            }
        }
    }

};


/*-------------------------------------------------------*/

class pointTestRes {

public:
    bool inside;
    pVector normal;
    pScalar depth;
};

class RigidBody {

public:
    pVector position;
    pVector linearVel;
    pVector forceAcc;

    pScalar angle;
    pScalar angularVel;
    pScalar torqueAcc;

    pScalar mass;
    pScalar inertia;

    Geometry geometry;

    Transform transform;

    bool fixed;

    pVector localToGlobal( pVector src );
    pVector globalToLocal( pVector src );

    pVector getAbsVelAtPoint( pVector globalPoint );

    void applyForceCM( pVector force );
    void applyTorque( pScalar torque );
    void applyForceAtPoint( pVector force, pVector globalPoint );
    void integrate(pScalar dt);

    pointTestRes pointTest( pVector point );

    void print();
};

pVector RigidBody::localToGlobal( pVector src ) {
    return transform.apply( src );
}

pVector RigidBody::globalToLocal( pVector src ) {
    return transform.applyInverse( src );
}

void RigidBody::integrate(pScalar dt) {

    //Integrate linear and angular velocities
    linearVel.x += (1/mass)*forceAcc.x * dt;
    linearVel.y += (1/mass)*forceAcc.y * dt;
    angularVel += (1/inertia)*torqueAcc * dt;

    //Integrate position
    if(!fixed) {
        position.x += linearVel.x * dt;
        position.y += linearVel.y * dt;
        angle += angularVel * dt;
    }

    forceAcc.x = 0.0;
    forceAcc.y = 0.0;
    torqueAcc = 0.0;

    //Update Rb's transform
    transform.set( angle, position );
}

void RigidBody::applyForceCM(pVector force) {
    forceAcc = add(forceAcc, force);
}

void RigidBody::applyTorque(pScalar torque) {
    torqueAcc += torque;
}

void RigidBody::applyForceAtPoint(pVector force, pVector globalPoint) {

    forceAcc = add(forceAcc, force);

    pVector localPoint = globalToLocal( globalPoint );

    //Calculate torque

    //Absolute value of cross product r x F for 2D case
    pScalar rfCrossAbs = ((localPoint.x*force.y)-(localPoint.y*force.x));

    //cout << "TORQUE:" << rfCrossAbs << endl;

    torqueAcc += rfCrossAbs;
}


void RigidBody::print() {
    cout << "[RIGIDBODY AT " << this << " ]" << endl;
    cout << "POSITION: ";
    position.print();
    cout << "LINEARVEL: ";
    linearVel.print();
    cout << "ANGLE: " << angle << endl;
    cout << "ANGLE VEL: " << angularVel << endl;
    cout << "[END]" << endl;
}

pointTestRes RigidBody::pointTest( pVector point ) {

    pVector localP = globalToLocal( point );

    unsigned int i = 0;

    pVector faceAxis;
    pScalar crossD = 0.0;
    unsigned int minAxisIndex = 0;
    pScalar minCrossAbs = 1000000.0;

    pointTestRes result;

    for( i = 0; i < geometry.points.size(); i++ ) {
        faceAxis = geometry.getFace( i );
        crossD = crossDir( faceAxis, sub( localP, geometry.points[i] ) );

        if( crossD > 0.0 ) {
            result.inside = false;
            return result;
        }

        if( fabs(crossD) < minCrossAbs ) {
            minCrossAbs = fabs(crossD);
            minAxisIndex = i;
        }
    }

    result.inside = true;
    result.normal = geometry.getNormal( minAxisIndex );
    result.depth = minCrossAbs / sub( localP, geometry.points[ minAxisIndex ] ).length();

    return result;
}

pVector RigidBody::getAbsVelAtPoint( pVector globalPoint ) {

    pVector V;// = linearVel;
    V.x = 0.0;
    V.y = 0.0;

    pVector point = sub( globalPoint, position );

    V = add( V, normal( point ).scale( point.length() * -angularVel ) );

    return V;
}

/*-------------------------------------------------------*/

//Face-vertex contact
//A contains face and B contatins vertex
class Contact {

public:
    RigidBody* A;	//Face body
    RigidBody* B;	//Vertex body
    pVector pos;	//Global coordianates
    pVector vel;	//? Later
    pVector normal; //Normal origin is at pB
    pScalar depth;	//depth of penetration

    void print() {
        cout << "[Contact: " << this << endl;
        cout << "[A: " << A << " B: " << B << " ]" << endl;
        cout << "POS:";
        pos.print();
        cout << "NOR:";
        normal.print();
        cout << "[DEPTH: " << depth << " ]" << endl;
    };
};

class Intersection {

public:
    pScalar distance;	//Depth of penetration
    int A_max_index;	//Indices of extremal faces
    int A_min_index;
    int B_max_index;
    int B_min_index;
    bool orderAB;		//Ordering: if true then P(cA) < P(cB)
};

/*-------------------------------------------------------*/

class World {

public:
    //std::pVector containing pointers to all rigid bodies which belong to this world
    vector<RigidBody*> bodyPtrs;

    void addBody(RigidBody* body);

    //Get std::pVector of pointers for contacts for all colliding bodies
    vector<Contact*>* getContacts();

    //Calculate intersection of projection of A and B to some unit axis
    Intersection projectRBaxis( RigidBody* A, RigidBody* B, pVector axis );

    //Collision reponse
    void resolveContacts(vector<Contact*>* contacts);

    //Surface handling
    void applySurfaceForces();

    //Vicious drag (speed dependent) improves stability
    void applyDrag();

    //Gravity
    void applyGravity();

    //Return Contact pointer if A and B collide, NULL otherwise
    vector<Contact*>* getContactList(RigidBody* A, RigidBody* B);
    vector<Contact*>* getContactListAlt(RigidBody* A, RigidBody* B);

    pScalar time;

    //Advance simulation by dt
    void timestep(pScalar dt);
};

void World::addBody(RigidBody* body) {
    bodyPtrs.push_back(body);
}

void vectorAppend( vector<Contact*>* dst, vector<Contact*>* src ) {
    unsigned int i = 0;

    for( i=0; i<src->size(); i++ ) {
        dst->push_back( src->at(i) );
    }
}

//If true then
bool radiusCollisionTest( RigidBody* A, RigidBody* B ) {
    return sub( A->position, B->position ).length() < ( A->geometry.radius + B->geometry.radius ) ;
}

//Collision detection
vector<Contact*>* World::getContacts() {

    //Result pVector
    vector<Contact*>* contactPtrs = new vector<Contact*>;

    vector<Contact*>* currentContactList = NULL;

    //Simple O(N^2) algorithm
    unsigned int i = 0;
    unsigned int j = 0;

    for( i=0; i<bodyPtrs.size(); i++ ) {
        for( j = 0; j<bodyPtrs.size(); j++ ) {

            //Check bounding circles
            if( (i != j) && radiusCollisionTest( bodyPtrs[i], bodyPtrs[j] ) ) {

                currentContactList = getContactListAlt(bodyPtrs[i], bodyPtrs[j]);

                if( currentContactList != NULL ) {

                    vectorAppend( contactPtrs, currentContactList );

                    delete currentContactList;

                }
            }
        }
    }

    return contactPtrs;
}

//Project convex rigid body's geometry to unit axis,
//return distance between boundaries of projections or negative value
//if there is overlap
Intersection World::projectRBaxis( RigidBody* A, RigidBody* B, pVector axis ) {

    //cout << axis.length() << endl;

    assert( fabs(axis.length() - 1.0) < 0.001 );

    pScalar A_max = scalarProduct( A->localToGlobal( A->geometry.points[0] ), axis );
    pScalar A_min = scalarProduct( A->localToGlobal( A->geometry.points[0] ), axis );
    pScalar B_max = scalarProduct( B->localToGlobal( B->geometry.points[0] ), axis );
    pScalar B_min = scalarProduct( B->localToGlobal( B->geometry.points[0] ), axis );

    int A_max_index = 0;
    int A_min_index = 0;
    int B_max_index = 0;
    int B_min_index = 0;

    pScalar A_centerp = 0.0;
    pScalar B_centerp = 0.0;
    pScalar proj = 0.0;

    bool order_AB = true;

    unsigned int i = 0;

    //Determines order of A and B relative to axis
    A_centerp = scalarProduct( A->position, axis );
    B_centerp = scalarProduct( B->position, axis );

    if( A < B )
        order_AB = true;
    else
        order_AB = false;

    //Project every face of A and B to axis and find min and max

    for(i = 0; i < A->geometry.points.size(); i++) {

        proj = scalarProduct( A->localToGlobal( A->geometry.points[i] ), axis );

        if( proj < A_min ) {
            A_min = proj;
            A_min_index = i;
        }
        if( proj > A_max ) {
            A_max = proj;
            A_max_index = i;
        }

    }

    for(i = 0; i < B->geometry.points.size(); i++) {

        proj = scalarProduct( B->localToGlobal( B->geometry.points[i] ), axis );

        if( proj < B_min ) {
            B_min = proj;
            B_min_index = i;
        }
        if( proj > B_max ) {
            B_max = proj;
            B_max_index = i;
        }
    }

    Intersection inter;

    //If A is on the left and B is on the right
    if( order_AB ) {
        //Either there is no overlap, then distance > 0
        //Or in the presence of overlap distance < 0
        //return (B_min - A_max);
        inter.distance = (B_min - A_max);
    } else {
        //If B is on the left, switch order of arguments
        //return (A_min - B_max);
        inter.distance = (A_min - B_max);
    }

    inter.A_max_index = A_max_index;
    inter.A_min_index = A_min_index;
    inter.B_max_index = B_max_index;
    inter.B_min_index = B_min_index;
    inter.orderAB = order_AB;

    return inter;
}

/*
bool trianglePointTest( pVector v1, pVector v2, pVector v3, pVector p ) {

}
*/

//Pair collision test
//For GEO_CONVEX only
vector<Contact*>* World::getContactList(RigidBody* A, RigidBody* B) {

    if( A == B ) {
        return NULL;
    }

    assert( A->geometry.type == GEO_CONVEX && B->geometry.type == GEO_CONVEX );

    //Contains potential projection axis for Separating Axis Theorem
    vector<pVector> cAxes;
    //cAxes.resize( A->geometry.points.size() + B->geometry.points.size() );

    unsigned int i = 0;

    //Push normal to every face of A and B to axis array
    for( i = 0; i < A->geometry.points.size(); i++ ) {
        cAxes.push_back( A->transform.applyRotation( A->geometry.getNormal( i ) ) );
    }

    for( i = 0; i < B->geometry.points.size(); i++) {
        cAxes.push_back( B->transform.applyRotation( B->geometry.getNormal( i ) ) );
    }

    Intersection inter;
    Intersection axisInter;

    pScalar minAbsOverlap = 1000000.0;
    pScalar distance = 0.0;

    unsigned int collisionAxisIndex = 0;
    pVector collisionAxis;

    RigidBody* axisBody = NULL;


    //Iterate over axes
    //Separating Axis theorem states that for two convex objects
    //to intersect means there is no one axis where their projections non-overlap
    //I.e if there is one axis where their projections are separate they do not overlap
    for(i = 0; i < cAxes.size(); i++) {

        inter = projectRBaxis( A, B, cAxes[i] );
        distance = inter.distance;

        //If there is one non-overlapping projection, there is no contact
        //TODO: add axis caching
        if( distance > 0.0 ) {
            return NULL;
        } else {
            if( fabs(distance) < minAbsOverlap ) {

                minAbsOverlap = fabs(distance);

                collisionAxisIndex = i;

                axisInter = inter;

            }
        }
    }

    collisionAxis = cAxes[ collisionAxisIndex ];

    if( collisionAxisIndex < A->geometry.points.size() ) {
        axisBody = A;
    } else {
        axisBody = B;
    }

    //Use saved information for creating Contact instance

    RigidBody* faceBody = NULL;
    RigidBody* vertexBody = NULL;

    pVector contactPos;

    if( axisBody == A ) {
        faceBody = A;
        vertexBody = B;
        contactPos = B->localToGlobal( B->geometry.points[ inter.B_max_index ] );
    } else {
        faceBody = B;
        vertexBody = A;
        contactPos = A->localToGlobal( A->geometry.points[ inter.A_max_index ] );
    }

    Contact* c = new Contact;

    c->A = faceBody;
    c->B = vertexBody;

    c->pos = contactPos;
    c->normal = collisionAxis;

    //A,B velocity here
    //...

    c->depth = minAbsOverlap;

    //To prevent memory leak
    cAxes.clear();

    vector<Contact*>* result = new vector<Contact*>;

    result->push_back(c);

    return result;
}

vector<Contact*>* World::getContactListAlt(RigidBody* A, RigidBody* B) {

    if( A == B ) {
        return NULL;
    }

    assert( A->geometry.type == GEO_CONVEX && B->geometry.type == GEO_CONVEX );

    unsigned int i = 0;

    vector<Contact*>* result = new vector<Contact*>;

    pointTestRes pt;

    for( i = 0; i < B->geometry.points.size(); i++ ) {

        pVector bPos = B->localToGlobal( B->geometry.points.at(i) );

        pt = A->pointTest( bPos );

        if( pt.inside ) {
            Contact* c = new Contact;

            c->A = A;
            c->B = B;
            c->vel = sub( B->getAbsVelAtPoint( bPos ), A->getAbsVelAtPoint( bPos ) );
            c->depth = pt.depth;
            c->normal = pt.normal;
            c->pos = bPos;

            result->push_back( c );
        }

    }

    if( result->empty() ) {
        delete result;
        return NULL;
    }

    return result;
}

vector<pVector> cNormals;
vector<pVector> cPosns;

//Contact resolution
void World::resolveContacts(vector<Contact*>* contacts) {

    pScalar k = 200.0;
    pScalar d = 1.0;

    Contact* contact = NULL;

    cNormals.clear();
    cPosns.clear();

    unsigned int i = 0;

    for( i=0; i<contacts->size(); i++ ) {

        //cout << "[N CONTACTS: " << contacts->size() << " ]" << endl;

        contact = contacts->at(i);
        /*
        cNormals.push_back( contact->normal );
        cPosns.push_back( contact->pos );

        cNormals.push_back( contact->vel );
        cPosns.push_back( contact->pos );
        */
        //contact->pos.print();

        //cout << "[" << contact->depth << "]" << endl;
        /*
        pVector Fa_n = contact->normal.scale(-k * contact->depth );
        pVector Fa_t = contact->vel.scale( d );

        //Fa_t.print();

        pVector Fb_n = contact->normal.scale( k * contact->depth );
        pVector Fb_t = contact->vel.scale( -d );
        */
        //contact->A->applyForceAtPoint( contact->pos, add( Fa_n, Fa_t ) );
        //contact->B->applyForceAtPoint( contact->pos, add( Fb_n, Fb_t ) );

        pScalar depth = contact->depth;

        pScalar forceAbs = k*fabs(depth);

        contact->A->applyForceAtPoint( contact->normal.scale(-forceAbs ), contact->pos );
        contact->B->applyForceAtPoint( contact->normal.scale( forceAbs ), contact->pos );

    }
}

void World::applySurfaceForces() {

    pScalar kSurface = 100.0;
    pScalar ySurface = 0.0;

    unsigned int i = 0;
    unsigned int j = 0;

    RigidBody* body = NULL;
    pScalar depth = 0.0;

    pVector force;
    pVector point;
    force.x = 0.0;
    force.y = 0.0;

    for( i=0; i < bodyPtrs.size(); i++ ) {

        body = bodyPtrs[i];

        for( j=0; j < body->geometry.points.size(); j++ ) {

            point = body->localToGlobal( body->geometry.points[j] );

            depth = ySurface - point.y;

            if( depth > 0.0 ) {
                force.y = kSurface * depth;
                body->applyForceAtPoint( force, point );
            }
        }
    }
}

void World::applyGravity() {

    unsigned int i = 0;
    RigidBody* body = NULL;
    pScalar gConstant = 1.0;
    pVector gForce;

    for( i=0; i < bodyPtrs.size(); i++ ) {

        body = bodyPtrs[i];

        gForce.x = 0.0;
        gForce.y = -gConstant * body->mass;

        body->applyForceCM( gForce );
    }
}

void World::applyDrag() {

    unsigned int i = 0;
    RigidBody* body = NULL;
    pScalar dragK = 1.0;

    for( i=0; i < bodyPtrs.size(); i++ ) {
        body = bodyPtrs[i];
        body->applyForceCM( body->linearVel.scale( -dragK ) );
        body->torqueAcc += ( -dragK * body->angularVel );
    }

}

void World::timestep( pScalar dT ) {

    time += dT;

    //Get list of contacts
    vector<Contact*>* contacts = getContacts();

    //Resolve them: apply forces
    resolveContacts( contacts );

    //Apply surface forces (e.g. ground)
    applySurfaceForces();

    applyGravity();

    applyDrag();

    //Integrate position of every body
    unsigned int i = 0;

    for( i=0; i<bodyPtrs.size(); i++ ) {
        bodyPtrs[i]->integrate( dT );
    }

    //Free contact list
    for( i=0; i<contacts->size(); i++ ) {
        delete contacts->at( i );
    }

    delete contacts;
}

#define PI 3.14159265

RigidBody* addCircle( World* world,
                      unsigned int N,
                      pScalar R,
                      pScalar x, pScalar y,
                      pScalar phi,
                      pScalar mass,
                      pScalar inertia ) {

    RigidBody* c = new RigidBody;

    c->position.x = x;
    c->position.y = y;
    c->angle = phi;
    c->mass = mass;
    c->inertia = inertia;

    c->fixed = false;

    c->linearVel.x = 0.0;
    c->linearVel.y = 0.0;
    c->angularVel  = 0.0;

    c->forceAcc.x = 0.0;
    c->forceAcc.y = 0.0;

    c->torqueAcc = 0.0;

    //Geometry
    c->geometry.type = GEO_CONVEX;

    unsigned int i = 0;

    for( i=0; i<N; i++ ) {

        //Clockwise!
        pScalar angle = -(2.0*PI*((double) i)/((double) N));

        pScalar x = R*cos(angle);
        pScalar y = R*sin(angle);

        pVector v;
        v.x = x;
        v.y = y;

        c->geometry.points.push_back( v );

    }

    c->geometry.updateRadius();

    world->bodyPtrs.push_back( c );

    return c;
}

//Test function, adds box to world
RigidBody* addBox( World* world, pScalar x,  pScalar y, pScalar w, pScalar h, pScalar phi, pScalar mass ) {

    RigidBody* box = new RigidBody;

    box->position.x = x;
    box->position.y = y;
    box->angle = phi;
    box->mass = mass;
    box->inertia = mass;

    box->linearVel.x = 0.0;
    box->linearVel.y = 0.0;
    box->angularVel  = 0.0;

    box->forceAcc.x = 0.0;
    box->forceAcc.y = 0.0;

    box->torqueAcc = 0.0;

    box->fixed = false;

    //Geometry
    box->geometry.type = GEO_CONVEX;

    pVector p;
    p.x = -w/2.0;
    p.y = -h/2.0;
    box->geometry.points.push_back( p );
    p.x = -w/2.0;
    p.y = h/2.0;
    box->geometry.points.push_back( p );
    p.x = w/2.0;
    p.y = h/2.0;
    box->geometry.points.push_back( p );
    p.x = w/2.0;
    p.y = -h/2.0;
    box->geometry.points.push_back( p );

    box->geometry.updateRadius();

    world->bodyPtrs.push_back( box );

    return box;
}

/*-------------------------------------------------------*/

bool physThreadRun = true;
bool stepPause = false;

unsigned int stepDelay = 0;

//Global world
World* w = new World;

void* physicsThread(void* p) {

    cout << "[PHYSICS: Starting..]" << endl;

    RigidBody* b2 = addBox( w, 0.0, -5.0, 1000.0, 16.0, 0.0, 10.0 );
    b2->fixed = true;

    //RigidBody* b2_ = addBox( w, 0.0, -5.0, 1000.0, 16.0, -0.5, 10.0 );
    //b2_->fixed = true;

    //RigidBody* b1 = addBox( w, 0.0, 16.0, 1.0, 1.0, 0.0, 1.0 );

    //RigidBody* b3 = addBox( w, 7.0, 16.0, 0.5, 2.0, 3.14, 1.2 );
    //RigidBody* b4 = addBox( w, 3.0, 16.0, 2.0, 2.0, 0.5, 1.0 );

    //RigidBody* b5 = addCircle( w, 5, 1.0, 0.0, 1.0, 1.0, 3.0, 4.0 );

    unsigned int i = 0;

    //Pyramid

    int N = 4;

    for( i=0; i<N; i++) {
        pScalar width = 1.0+(N-i)*1.0;
        addBox( w, 0.0, 3.5+i, width, 1.0, 0.0, width*1.0);
    }
    /*
    RigidBody* rocket_A = addBox( w, 5.0, 6.0, 1.0, 1.0, 0.1, 10.0 );
    rocket_A->linearVel.x = -5.0;
    rocket_A->angularVel = 3.0;
    */
    /*
    RigidBody* rocket_B = addBox( w, -5.0, 3.0, 2.0, 2.0, -0.1, 12.0 );
    rocket_B->linearVel.x = 80.0;
    rocket_B->angularVel = -4.0;
    */
    /*
    //House
    int N = 10;
    pScalar hWidth = 6.0;

    for( i=0; i<N; i++) {

        pScalar originX = 3.0+(((pScalar)i)*4.5);

        addBox( w, 10, 4.0+i*7.0, 6.0, 1.0, 0.0, 6.0);
        //addBox( w, 8,  originX+1.5, 1.0, 3.0, 0.0, 3.0);
        //addBox( w, 12, originX+1.5, 1.0, 3.0, 0.0, 3.0);


    }
    */
    //cout << w->bodyPtrs.size() << endl;

    unsigned int time = 0;
    unsigned int printDelta = 100;

    while( physThreadRun ) {

        Sleep( stepDelay );

        if( stepPause ) {

            Sleep( 10 );

        } else {

            Sleep( 9 );

            w->timestep( 0.01 );

        }
    }

    cout << "[PHYSICS: shutdown]" << endl;

    return NULL;
}

//-------------------------------------------------------------------------------------//
//  GRAPHICS

bool graphicsThreadRun = true;

Transform camTransform;

void moveCam( pVector pos ) {
    camTransform.offset = pos.scale(-1.0);
}

void moveCamRel( pVector dPos ) {
    camTransform.offset = add( camTransform.offset, dPos.scale(-1.0) );
}

void scaleCam( pVector scale ) {
    camTransform.a11 = scale.x;
    camTransform.a22 = -scale.y;
}

pVector getScaleCam() {
    pVector result;
    result.x = camTransform.a11;
    result.y = -camTransform.a22;
    return result;
}

void camInit() {
    camTransform.a11 = 30.0;
    camTransform.a22 = -30.0;

    camTransform.offset.x = SCREEN_WIDTH*0.5;
    camTransform.offset.y = SCREEN_HEIGHT*0.5;
}

//Global screen
SDL_Surface* screen = NULL;

void setPixel(int x, int y, Uint32 pixel) {

    if( x > SCREEN_WIDTH ||
            x < 0 ||
            y > SCREEN_HEIGHT ||
            y < 0 ) {
        return;
    }

    SDL_Surface* surface = screen;
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to set */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        *p = pixel;
        break;

    case 2:
        *(Uint16 *)p = pixel;
        break;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
            p[0] = (pixel >> 16) & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = pixel & 0xff;
        } else {
            p[0] = pixel & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = (pixel >> 16) & 0xff;
        }
        break;

    case 4:
        *(Uint32 *)p = pixel;
        break;
    }
}

void drawLine(int x1, int y1, int x2, int y2, Uint32 color) {

    const int deltaX = abs(x2 - x1);
    const int deltaY = abs(y2 - y1);
    const int signX = x1 < x2 ? 1 : -1;
    const int signY = y1 < y2 ? 1 : -1;

    int error = deltaX - deltaY;

    setPixel(x2, y2, color);

    //setPixel(x1, y1-1, color);
    //setPixel(x1, y1+1, color);
    //setPixel(x1-1, y1, color);
    //setPixel(x1+1, y1, color);

    while(x1 != x2 || y1 != y2) {

        setPixel(x1, y1-1, color);
        setPixel(x1, y1+1, color);
        setPixel(x1-1, y1, color);
        setPixel(x1+1, y1, color);
        setPixel(x1,y1,color);

        const int error2 = error * 2;

        if(error2 > -deltaY) {
            error -= deltaY;
            x1 += signX;
        }
        if(error2 < deltaX) {
            error += deltaX;
            y1 += signY;
        }
    }
}

//Draw line with two vectors respecting camera transform
void drawLineVV( pVector A, pVector B ) {

    pVector At = camTransform.apply( A );
    pVector Bt = camTransform.apply( B );

    /*
    A.print();
    At.print();
    B.print();
    Bt.print();
    */

    /*

    if( (int) At.x < 0 ||
        (int) At.y < 0 ||
        (int) Bt.x < 0 ||
        (int) Bt.y < 0 ) {

        cout << "[DRAW NEGATIVE: " << (int) A.x << (int) A.y << (int) B.x << (int) B.y << " ]" << endl;

        return;
    }

    */

    drawLine((int) At.x,
             (int) At.y,
             (int) Bt.x,
             (int) Bt.y, 0x00EE1111);


    // cout << "[DRAW: " << (int) A.x << (int) A.y << (int) B.x << (int) B.y << " ]" << endl;
    // cout << "[DRAW: " << (int) At.x << " ]" << endl;

}

void drawLineVV( pVector A, pVector B, Uint32 color ) {

    pVector At = camTransform.apply( A );
    pVector Bt = camTransform.apply( B );

    drawLine((int) At.x,
             (int) At.y,
             (int) Bt.x,
             (int) Bt.y, color);
}

void drawVector( pVector pos, pVector v ) {
    drawLineVV( pos, add( pos, v ), 0x0011EE11 );
}

void drawRB( RigidBody* RB ) {

    unsigned int i = 0;
    unsigned int length = RB->geometry.points.size();

    pVector temp1;
    pVector temp2;

    for( i=0; i < length-1; i++ ) {
        temp1 = RB->localToGlobal( RB->geometry.points[i] );
        temp2 = RB->localToGlobal( RB->geometry.points[i+1] );
        drawLineVV( temp1, temp2 );
    }

    temp1 = RB->localToGlobal(  RB->geometry.points[ length-1 ] );
    temp2 = RB->localToGlobal( RB->geometry.points[0] );
    drawLineVV( temp1, temp2 );

}

TTF_Font* loadfont(char* file, int ptsize) {
    TTF_Font* tmpfont;
    tmpfont = TTF_OpenFont(file, ptsize);
    if (tmpfont == NULL) {
        printf("Unable to load font: %s %s \n", file, TTF_GetError());
        // Handle the error here.
    }
    return tmpfont;
}

enum textquality {solid, shaded, blended};

SDL_Surface* drawtext(TTF_Font *fonttodraw, char fgR, char fgG, char fgB, char fgA,
                      char bgR, char bgG, char bgB, char bgA, char text[], textquality quality) {
    SDL_Color tmpfontcolor = {fgR,fgG,fgB,fgA};
    SDL_Color tmpfontbgcolor = {bgR, bgG, bgB, bgA};
    SDL_Surface *resulting_text;

    if (quality == solid) resulting_text = TTF_RenderText_Solid(fonttodraw, text, tmpfontcolor);
    else if (quality == shaded) resulting_text = TTF_RenderText_Shaded(fonttodraw, text, tmpfontcolor, tmpfontbgcolor);
    else if (quality == blended) resulting_text = TTF_RenderText_Blended(fonttodraw, text, tmpfontcolor);

    return resulting_text;
}

//TTF_Font* monoFont = NULL;

//http://content.gpwiki.org/index.php/SDL_ttf:Tutorials:Basic_Font_Rendering

void* graphicsThread( void* arg ) {

    cout << "[GRAPHICS: Starting..]" << endl;

    Uint32 bgColor = 0x00FFFF00;

    camInit();

    // initialize SDL video
    if ( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
        printf( "[Unable to init SDL: %s]\n", SDL_GetError() );
        return NULL;
    }

    // make sure SDL cleans up before exit
    atexit(SDL_Quit);

    // create a new window
    screen = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 32,
                              SDL_HWSURFACE|SDL_DOUBLEBUF);
    if ( !screen ) {
        printf("Unable to set SCREEN_WIDTHxSCREEN_HEIGHT video: %s\n", SDL_GetError());
        return NULL;
    }

    if (TTF_Init() == -1) {
        printf("Unable to initialize SDL_ttf: %s \n", TTF_GetError());
    }

    //monoFont = loadfont("DroidSansMono.ttf", 16);

    pScalar moveConst = 1.0;

    while ( graphicsThreadRun ) {
        // message processing loop
        SDL_Event event;

        pVector offset;
        pVector scale;

        while (SDL_PollEvent(&event)) {
            // check for messages
            switch (event.type) {
                // exit if the window is closed
            case SDL_QUIT:
                graphicsThreadRun = false;
                physThreadRun = false;
                break;

                // check for keypresses
            case SDL_KEYDOWN: {
                // exit if ESCAPE is pressed
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    graphicsThreadRun = false;
                    physThreadRun = false;
                    break;
                }

                if (event.key.keysym.sym == SDLK_UP ) {

                    scale = getScaleCam();

                    offset.x = 0.0;
                    offset.y = -moveConst * scale.y;

                    moveCamRel( offset );
                }

                if (event.key.keysym.sym == SDLK_DOWN ) {

                    scale = getScaleCam();

                    offset.x = 0.0;
                    offset.y = moveConst * scale.y;
                    moveCamRel( offset );
                }

                if (event.key.keysym.sym == SDLK_LEFT ) {

                    scale = getScaleCam();

                    offset.x = -moveConst * scale.x;
                    offset.y = 0.0;
                    moveCamRel( offset );
                }

                if (event.key.keysym.sym == SDLK_RIGHT ) {

                    scale = getScaleCam();

                    offset.x = moveConst * scale.x;
                    offset.y = 0.0;
                    moveCamRel( offset );
                }

                if (event.key.keysym.sym == SDLK_F1 ) {

                    if( stepDelay == 0 ) {
                        stepDelay = 1;
                    } else {
                        stepDelay *= 2;
                    }

                    cout << "[STEP DELAY: " << stepDelay << " ]" << endl;

                }

                if (event.key.keysym.sym == SDLK_F2 ) {

                    if( stepDelay == 0 ) {
                        stepDelay = 0;
                    } else {
                        stepDelay /= 2;
                    }

                    cout << "[STEP DELAY: " << stepDelay << " ]" << endl;

                }

                if (event.key.keysym.sym == SDLK_SPACE ) {

                    stepPause = !stepPause;
                    cout << "[PAUSE " << stepPause << " ]" << endl;

                }

                break;
            }

            case SDL_MOUSEBUTTONDOWN: {
                if( event.button.button == 4 ) {

                    scale = getScaleCam();

                    pVector temp;
                    temp.x = 0.8;
                    temp.y = 0.8;

                    temp.x *= scale.x;
                    temp.y *= scale.y;

                    scaleCam( temp );

                }

                if( event.button.button == 5 ) {

                    scale = getScaleCam();

                    pVector temp;
                    temp.x = 1.2;
                    temp.y = 1.2;

                    temp.x *= scale.x;
                    temp.y *= scale.y;

                    scaleCam( temp );

                }


                break;
            }
            }
        }

        //DRAWING STARTS HERE

        //Clear screen
        SDL_FillRect(screen, 0, SDL_MapRGB( screen->format, 0xC0, 0xF8, 0xD0 ));

        unsigned int i = 0;

        //cout << "N:" << w->bodyPtrs.size() << endl;

        for( i=0; i<w->bodyPtrs.size(); i++ ) {

            drawRB( w->bodyPtrs.at(i) );

        }

        /*
        for( i=0; i<cNormals.size(); i++ ) {

            drawVector( cPosns[i], cNormals[i] );

        }
        */
        Sleep(10);


        // DRAWING ENDS HERE

        //finally, update the screen :)
        SDL_Flip( screen );
    } //end main loop

    cout << "[GRAPHICS: shutdown]" << endl;

    return NULL;

}

int main() {
    pthread_t T1, T2;
    int RET1, RET2;

    cout << "[MAIN: Starting..]" << endl;

    RET1 = pthread_create( &T1, NULL, physicsThread, (void*) NULL);
    RET2 = pthread_create( &T2, NULL, graphicsThread, (void*) NULL);

    graphicsThreadRun = true;
    pthread_join( T2, NULL );

    physThreadRun = true;
    pthread_join( T1, NULL );

    return 0;
}


int SDL_main(int argc, char* argv[]) {

    freopen("CON", "w", stdout);
    freopen("CON", "w", stderr);

    main();

    return 0;
}

