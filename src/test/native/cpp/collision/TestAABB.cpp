#include <gtest/gtest.h>
#include <reactphysics3d/reactphysics3d.h>
using namespace rp3d;

class CollisionAABB : public testing::Test {
protected:
  AABB mAABB1;
  AABB mAABB2;
  AABB mAABB3;
  AABB mAABB4;

  void SetUp() override {
    mAABB1.setMin(Vector3(-10, -10, -10));
    mAABB1.setMax(Vector3(10, 10, 10));

    // AABB2 intersect with AABB1
    mAABB2.setMin(Vector3(-5, 4, -30));
    mAABB2.setMax(Vector3(-2, 20, 30));

    // AABB3 contains AABB1
    mAABB3.setMin(Vector3(-25, -25, -25));
    mAABB3.setMax(Vector3(25, 25, 25));

    // AABB4 does not collide with AABB1
    mAABB4.setMin(Vector3(-40, -40, -40));
    mAABB4.setMax(Vector3(-15, -25, -12));
  }
};

TEST_F(CollisionAABB, BasicMethods) {
  AABB aabb1;
  AABB aabb2(Vector3(-3, -5, -8), Vector3(65, -1, 56));
  Vector3 trianglePoints[] = {
      Vector3(-5, 7, 23), Vector3(45, -34, -73), Vector3(-12, 98, 76)
  };
  AABB aabb3 = AABB::createAABBForTriangle(trianglePoints);

  EXPECT_EQ(aabb1.getMin().x, 0);
  EXPECT_EQ(aabb1.getMin().y, 0);
  EXPECT_EQ(aabb1.getMin().z, 0);
  EXPECT_EQ(aabb1.getMax().x, 0);
  EXPECT_EQ(aabb1.getMax().y, 0);
  EXPECT_EQ(aabb1.getMax().z, 0);

  EXPECT_EQ(aabb2.getMin().x, -3);
  EXPECT_EQ(aabb2.getMin().y, -5);
  EXPECT_EQ(aabb2.getMin().z, -8);
  EXPECT_EQ(aabb2.getMax().x, 65);
  EXPECT_EQ(aabb2.getMax().y, -1);
  EXPECT_EQ(aabb2.getMax().z, 56);

  EXPECT_EQ(aabb3.getMin().x, -12);
  EXPECT_EQ(aabb3.getMin().y, -34);
  EXPECT_EQ(aabb3.getMin().z, -73);
  EXPECT_EQ(aabb3.getMax().x, 45);
  EXPECT_EQ(aabb3.getMax().y, 98);
  EXPECT_EQ(aabb3.getMax().z, 76);

  // -------- Test inflate() -------- //
  AABB aabbInflate(Vector3(-3, 4, 8), Vector3(-1, 6, 32));
  aabbInflate.inflate(1, 2, 3);
  EXPECT_NEAR(aabbInflate.getMin().x, -4, 0.00001);
  EXPECT_NEAR(aabbInflate.getMin().y, 2, 0.00001);
  EXPECT_NEAR(aabbInflate.getMin().z, 5, 0.00001);
  EXPECT_NEAR(aabbInflate.getMax().x, 0, 0.00001);
  EXPECT_NEAR(aabbInflate.getMax().y, 8, 0.00001);
  EXPECT_NEAR(aabbInflate.getMax().z, 35, 0.00001);

  // -------- Test getExtent() --------- //

  EXPECT_FLOAT_EQ(mAABB1.getExtent().x, 20);
  EXPECT_FLOAT_EQ(mAABB1.getExtent().y, 20);
  EXPECT_FLOAT_EQ(mAABB1.getExtent().z, 20);

  EXPECT_FLOAT_EQ(mAABB2.getExtent().x, 3);
  EXPECT_FLOAT_EQ(mAABB2.getExtent().y, 16);
  EXPECT_FLOAT_EQ(mAABB2.getExtent().z, 60);

  EXPECT_FLOAT_EQ(mAABB3.getExtent().x, 50);
  EXPECT_FLOAT_EQ(mAABB3.getExtent().y, 50);
  EXPECT_FLOAT_EQ(mAABB3.getExtent().z, 50);

  // -------- Test getCenter() -------- //

  EXPECT_EQ(mAABB1.getCenter().x, 0);
  EXPECT_EQ(mAABB1.getCenter().y, 0);
  EXPECT_EQ(mAABB1.getCenter().z, 0);

  EXPECT_FLOAT_EQ(mAABB2.getCenter().x, -3.5);
  EXPECT_FLOAT_EQ(mAABB2.getCenter().y, 12);
  EXPECT_FLOAT_EQ(mAABB2.getCenter().z, 0);

  // -------- Test setMin(), setMax(), getMin(), getMax() -------- //

  AABB aabb5;
  aabb5.setMin(Vector3(-12, 34, 6));
  aabb5.setMax(Vector3(-3, 56, 20));

  EXPECT_EQ(aabb5.getMin().x, -12);
  EXPECT_EQ(aabb5.getMin().y, 34);
  EXPECT_EQ(aabb5.getMin().z, 6);
  EXPECT_EQ(aabb5.getMax().x, -3);
  EXPECT_EQ(aabb5.getMax().y, 56);
  EXPECT_EQ(aabb5.getMax().z, 20);

  // -------- Test assignment operator -------- //

  AABB aabb6;
  aabb6 = aabb2;

  EXPECT_EQ(aabb6.getMin().x, -3);
  EXPECT_EQ(aabb6.getMin().y, -5);
  EXPECT_EQ(aabb6.getMin().z, -8);
  EXPECT_EQ(aabb6.getMax().x, 65);
  EXPECT_EQ(aabb6.getMax().y, -1);
  EXPECT_EQ(aabb6.getMax().z, 56);

  // -------- Test getVolume() -------- //

  EXPECT_FLOAT_EQ(mAABB1.getVolume(), 8000);
  EXPECT_FLOAT_EQ(mAABB2.getVolume(), 2880);

  // -------- Test applyScale() -------- //

  AABB aabb7(Vector3(1,2,3), Vector3(5, 6, 7));
  aabb7.applyScale(Vector3(1, 2, 3));

  EXPECT_FLOAT_EQ(aabb7.getMin().x, 1);
  EXPECT_FLOAT_EQ(aabb7.getMin().y, 4);
  EXPECT_FLOAT_EQ(aabb7.getMin().z, 9);
  EXPECT_FLOAT_EQ(aabb7.getMax().x, 5);
  EXPECT_FLOAT_EQ(aabb7.getMax().y, 12);
  EXPECT_FLOAT_EQ(aabb7.getMax().z, 21);
}

TEST_F(CollisionAABB, MergeMethods) {
  AABB aabb1(Vector3(-45, 7, -2), Vector3(23, 8, 1));
  AABB aabb2(Vector3(-15, 6, 23), Vector3(-5, 9, 45));

  // -------- Test mergeTwoAABBs() -------- //

  AABB aabb3;
  aabb3.mergeTwoAABBs(aabb1, mAABB1);

  EXPECT_EQ(aabb3.getMin().x, -45);
  EXPECT_EQ(aabb3.getMin().y, -10);
  EXPECT_EQ(aabb3.getMin().z, -10);
  EXPECT_EQ(aabb3.getMax().x, 23);
  EXPECT_EQ(aabb3.getMax().y, 10);
  EXPECT_EQ(aabb3.getMax().z, 10);

  AABB aabb4;
  aabb4.mergeTwoAABBs(aabb1, aabb2);

  EXPECT_EQ(aabb4.getMin().x, -45);
  EXPECT_EQ(aabb4.getMin().y, 6);
  EXPECT_EQ(aabb4.getMin().z, -2);
  EXPECT_EQ(aabb4.getMax().x, 23);
  EXPECT_EQ(aabb4.getMax().y, 9);
  EXPECT_EQ(aabb4.getMax().z, 45);

  // -------- Test mergeWithAABB() -------- //

  aabb1.mergeWithAABB(mAABB1);

  EXPECT_EQ(aabb1.getMin().x, -45);
  EXPECT_EQ(aabb1.getMin().y, -10);
  EXPECT_EQ(aabb1.getMin().z, -10);
  EXPECT_EQ(aabb1.getMax().x, 23);
  EXPECT_EQ(aabb1.getMax().y, 10);
  EXPECT_EQ(aabb1.getMax().z, 10);

  aabb2.mergeWithAABB(mAABB1);

  EXPECT_EQ(aabb2.getMin().x, -15);
  EXPECT_EQ(aabb2.getMin().y, -10);
  EXPECT_EQ(aabb2.getMin().z, -10);
  EXPECT_EQ(aabb2.getMax().x, 10);
  EXPECT_EQ(aabb2.getMax().y, 10);
  EXPECT_EQ(aabb2.getMax().z, 45);
}

TEST_F(CollisionAABB, Intersection) {
  // -------- Test contains(AABB) -------- //
  EXPECT_FALSE(mAABB1.contains(mAABB2));
  EXPECT_TRUE(mAABB3.contains(mAABB1));
  EXPECT_FALSE(mAABB1.contains(mAABB3));
  EXPECT_FALSE(mAABB1.contains(mAABB4));
  EXPECT_FALSE(mAABB4.contains(mAABB1));

  // -------- Test contains(Vector3) -------- //
  EXPECT_TRUE(mAABB1.contains(Vector3(0, 0, 0)));
  EXPECT_TRUE(mAABB1.contains(Vector3(-5, 6, 9)));
  EXPECT_TRUE(mAABB1.contains(Vector3(-9, -4, -9)));
  EXPECT_TRUE(mAABB1.contains(Vector3(9, 4, 7)));
  EXPECT_FALSE(mAABB1.contains(Vector3(-11, -4, -9)));
  EXPECT_FALSE(mAABB1.contains(Vector3(1, 12, -9)));
  EXPECT_FALSE(mAABB1.contains(Vector3(1, 8, -13)));
  EXPECT_FALSE(mAABB1.contains(Vector3(-14, 82, -13)));

  // -------- Test testCollision() -------- //
  EXPECT_TRUE(mAABB1.testCollision(mAABB2));
  EXPECT_TRUE(mAABB2.testCollision(mAABB1));
  EXPECT_TRUE(mAABB1.testCollision(mAABB3));
  EXPECT_TRUE(mAABB3.testCollision(mAABB1));
  EXPECT_FALSE(mAABB1.testCollision(mAABB4));
  EXPECT_FALSE(mAABB4.testCollision(mAABB1));

  // -------- Test testCollisionTriangleAABB() -------- //

  AABB aabb(Vector3(100, 100, 100), Vector3(200, 200, 200));
  Vector3 trianglePoints[] = {
      Vector3(-2, 4, 6), Vector3(20, -34, -73), Vector3(-12, 98, 76)
  };
  EXPECT_TRUE(mAABB1.testCollisionTriangleAABB(trianglePoints));
  EXPECT_FALSE(aabb.testCollisionTriangleAABB(trianglePoints));

  // -------- Test testRayIntersect() -------- //

  Ray ray1(Vector3(-20, 4, -7), Vector3(20, 4, -7));
  Ray ray2(Vector3(-20, 11, -7), Vector3(20, 11, -7));
  Ray ray3(Vector3(0, 15, 0), Vector3(0, -15, 0));
  Ray ray4(Vector3(0, -15, 0), Vector3(0, 15, 0));
  Ray ray5(Vector3(-3, 4, 8), Vector3(-7, 9, 4));
  Ray ray6(Vector3(-4, 6, -100), Vector3(-4, 6, -9));
  Ray ray7(Vector3(-4, 6, -100), Vector3(-4, 6, -11), 0.6f);
  Ray ray8(Vector3(-403, -432, -100), Vector3(134, 643, 23));

  const Vector3 ray1Direction = ray1.point2 - ray1.point1;
  const Vector3 ray1DirectionInv(decimal(1.0) / ray1Direction.x, decimal(1.0) / ray1Direction.y, decimal(1.0) / ray1Direction.z);
  const Vector3 ray2Direction = ray2.point2 - ray2.point1;
  const Vector3 ray2DirectionInv(decimal(1.0) / ray2Direction.x, decimal(1.0) / ray2Direction.y, decimal(1.0) / ray2Direction.z);
  const Vector3 ray3Direction = ray3.point2 - ray3.point1;
  const Vector3 ray3DirectionInv(decimal(1.0) / ray3Direction.x, decimal(1.0) / ray3Direction.y, decimal(1.0) / ray3Direction.z);
  const Vector3 ray4Direction = ray4.point2 - ray4.point1;
  const Vector3 ray4DirectionInv(decimal(1.0) / ray4Direction.x, decimal(1.0) / ray4Direction.y, decimal(1.0) / ray4Direction.z);
  const Vector3 ray5Direction = ray5.point2 - ray5.point1;
  const Vector3 ray5DirectionInv(decimal(1.0) / ray5Direction.x, decimal(1.0) / ray5Direction.y, decimal(1.0) / ray5Direction.z);
  const Vector3 ray6Direction = ray6.point2 - ray6.point1;
  const Vector3 ray6DirectionInv(decimal(1.0) / ray6Direction.x, decimal(1.0) / ray6Direction.y, decimal(1.0) / ray6Direction.z);
  const Vector3 ray7Direction = ray7.point2 - ray7.point1;
  const Vector3 ray7DirectionInv(decimal(1.0) / ray7Direction.x, decimal(1.0) / ray7Direction.y, decimal(1.0) / ray7Direction.z);
  const Vector3 ray8Direction = ray8.point2 - ray8.point1;
  const Vector3 ray8DirectionInv(decimal(1.0) / ray8Direction.x, decimal(1.0) / ray8Direction.y, decimal(1.0) / ray8Direction.z);

  EXPECT_TRUE(mAABB1.testRayIntersect(ray1.point1, ray1DirectionInv, decimal(1.0)));
  EXPECT_FALSE(mAABB1.testRayIntersect(ray2.point1, ray2DirectionInv, decimal(1.0)));
  EXPECT_TRUE(mAABB1.testRayIntersect(ray3.point1, ray3DirectionInv, decimal(1.0)));
  EXPECT_TRUE(mAABB1.testRayIntersect(ray4.point1, ray4DirectionInv, decimal(1.0)));
  EXPECT_TRUE(mAABB1.testRayIntersect(ray5.point1, ray5DirectionInv, decimal(1.0)));
  EXPECT_TRUE(mAABB1.testRayIntersect(ray6.point1, ray6DirectionInv, decimal(1.0)));
  EXPECT_FALSE(mAABB1.testRayIntersect(ray7.point1, ray7DirectionInv, decimal(1.0)));
  EXPECT_FALSE(mAABB1.testRayIntersect(ray8.point1, ray8DirectionInv, decimal(1.0)));
}