#ifndef BBOX
#define BBOX

#include <openGL/drawable.h>
#include <la.h>

class Geometry;
class Intersection;
class Ray;


// object oriented boundig box
class BBox: public Drawable
{
public:

    BBox();
    BBox(const glm::vec3 &minB, const glm::vec3 &maxB);
    BBox(const BBox &bb);

    glm::vec3 getCenterpoint();

    // set get
    glm::vec3 getMinBounding() const;
    glm::vec3 getMaxBoudning() const;
    void setMinBounding(glm::vec3 &mb);
    void setMaxBounding(glm::vec3 &mb);

    void reComputeBoundingBox(const glm::mat4 &T);
    std::vector<glm::vec3> bBoxPos;

    //Methods inherited from Drawable
    void create();
    virtual GLenum drawMode();

    static glm::vec3 mMinVector(glm::vec3 &a, glm::vec3 &b);
    static glm::vec3 mMinVector(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c);
    static glm::vec3 mMaxVector(glm::vec3 &a, glm::vec3 &b);
    static glm::vec3 mMaxVector(glm::vec3 &a, glm::vec3 &b, glm::vec3 &c);

    //Geometry *geometryAttached;
    glm::vec3 minBounding;
    glm::vec3 maxBounding;

    Intersection getIntersection(const Ray &r);

    glm::vec3 boxColor;
};

#endif // BBOX

