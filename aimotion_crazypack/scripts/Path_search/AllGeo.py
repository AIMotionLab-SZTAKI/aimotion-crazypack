class GeoFace(object):

    # ptList: vertice GeoPoint Array
    # idxList: vertice index Integer Array
    def __init__(self, ptList, idxList):
        # alloc instance array memory
        self.__v = []  # vertice point array
        self.__idx = []  # vertice index array

        self.__n = len(ptList)  # number of vertices

        for i in range(0, self.__n):
            self.__v.append(ptList[i])
            self.__idx.append(idxList[i])

    @property
    def v(self):
        return self.__v

    @property
    def idx(self):
        return self.__idx

    @property
    def n(self):
        return self.__n
#-----------------------------------------------------------------------------------------------------------------------


class GeoPlane(object):

    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    @staticmethod
    def Create(p0, p1, p2):  # p0, p1, p2: GeoPoint

        v = GeoVector(p0, p1)
        u = GeoVector(p0, p2)

        # normal vector
        n = u * v

        a = n.x
        b = n.y
        c = n.z
        d = - (a * p0.x + b * p0.y + c * p0.z)

        return GeoPlane(a, b, c, d)

    def __neg__(self):
        return GeoPlane(-self.a, -self.b, -self.c, -self.d)

    def __mul__(self, pt):  # pt: GeoPoint
        return self.a * pt.x + self.b * pt.y + self.c * pt.z + self.d
#-----------------------------------------------------------------------------------------------------------------------


class GeoPoint(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, p):
        return GeoPoint(self.x + p.x, self.y + p.y, self.z + p.z)
#-----------------------------------------------------------------------------------------------------------------------


class GeoPolygon(object):

    def __init__(self, ptList):  # ptList: vertice GeoPoint Array

        # alloc instance array memory
        self.__v = []  # vertice point array
        self.__idx = []  # vertice index array

        self.__n = len(ptList)  # number of vertices

        for pt in ptList:
            self.__v.append(pt)
            self.__idx.append(ptList.index(pt))

    @property
    def v(self):
        return self.__v

    @property
    def idx(self):
        return self.__idx

    @property
    def n(self):
        return self.__n
#-----------------------------------------------------------------------------------------------------------------------


class GeoPolygonProc(object):

    def __init__(self, polygonInst):  # polygonInst: GeoPolygon

        self.__MaxUnitMeasureError = 0.001

        # Set boundary
        self.__Set3DPolygonBoundary(polygonInst)

        # Set maximum point to face plane distance error,
        self.__Set3DPolygonUnitError()

        # Set faces and face planes
        self.__SetConvex3DFaces(polygonInst)

    @property
    def x0(self):
        return self.__x0

    @property
    def x1(self):
        return self.__x1

    @property
    def y0(self):
        return self.__y0

    @property
    def y1(self):
        return self.__y1

    @property
    def z0(self):
        return self.__z0

    @property
    def z1(self):
        return self.__z1

    @property
    def NumberOfFaces(self):
        return self.__NumberOfFaces

    @property
    def FacePlanes(self):
        return self.__FacePlanes

    @property
    def Faces(self):
        return self.__Faces

    def __Set3DPolygonBoundary(self, polygon):  # polygon: GeoPolygon

        # List<GeoPoint>
        vertices = polygon.v

        n = polygon.n

        xmin = vertices[0].x
        xmax = vertices[0].x
        ymin = vertices[0].y
        ymax = vertices[0].y
        zmin = vertices[0].z
        zmax = vertices[0].z

        for i in range(1, n):

            if vertices[i].x < xmin:
                xmin = vertices[i].x
            if vertices[i].y < ymin:
                ymin = vertices[i].y
            if vertices[i].z < zmin:
                zmin = vertices[i].z
            if vertices[i].x > xmax:
                xmax = vertices[i].x
            if vertices[i].y > ymax:
                ymax = vertices[i].y
            if vertices[i].z > zmax:
                zmax = vertices[i].z

        self.__x0 = xmin
        self.__x1 = xmax
        self.__y0 = ymin
        self.__y1 = ymax
        self.__z0 = zmin
        self.__z1 = zmax

    def __Set3DPolygonUnitError(self):
        self.MaxDisError = ((abs(self.__x0) + abs(self.__x1) +
                             abs(self.__y0) + abs(self.__y1) +
                             abs(self.__z0) + abs(self.__z1)) / 6 * self.__MaxUnitMeasureError)

    def __SetConvex3DFaces(self, polygon):  # polygon: GeoPolygon

        # vertices indexes for all faces, 2d list
        faceVerticeIndex = []

        # face planes for all faces, 1d list
        fpOutward = []

        vertices = polygon.v
        n = polygon.n

        for i in range(0, n):

            # triangle point 1

            p0 = vertices[i]

            for j in range(i + 1, n):

                # triangle p
                p1 = vertices[j]

                for k in range(j + 1, n):

                    # triangle point 3

                    p2 = vertices[k]

                    trianglePlane = GeoPlane.Create(p0, p1, p2)

                    onLeftCount = 0
                    onRightCount = 0

                    # indexes of points that lie in same plane with face triangle plane
                    pointInSamePlaneIndex = []

                    for l in range(0, n):

                        # any point other than the 3 triangle points
                        if l != i and l != j and l != k:

                            pt = vertices[l]

                            dis = trianglePlane * pt

                            # next point is in the triangle plane
                            if abs(dis) < self.MaxDisError:
                                pointInSamePlaneIndex.append(l)
                            else:
                                if dis < 0:
                                    onLeftCount = onLeftCount + 1
                                else:
                                    onRightCount = onRightCount + 1

                                    # This is a face for a CONVEX 3d polygon.
                    # For a CONCAVE 3d polygon, this maybe not a face.
                    if onLeftCount == 0 or onRightCount == 0:

                        verticeIndexInOneFace = []

                        # triangle plane
                        verticeIndexInOneFace.append(i)
                        verticeIndexInOneFace.append(j)
                        verticeIndexInOneFace.append(k)

                        m = len(pointInSamePlaneIndex)

                        if m > 0:  # there are other vertices in this triangle plane
                            for p in range(0, m):
                                verticeIndexInOneFace.append(pointInSamePlaneIndex[p])

                        # if verticeIndexInOneFace is a new face
                        if not Utility.ContainsList(faceVerticeIndex, verticeIndexInOneFace):

                            # print(verticeIndexInOneFace)

                            # add it in the faceVerticeIndex list
                            faceVerticeIndex.append(verticeIndexInOneFace)

                            # add the trianglePlane in the face plane list fpOutward.
                            if onRightCount == 0:
                                fpOutward.append(trianglePlane)
                            else:
                                if onLeftCount == 0:
                                    fpOutward.append(-trianglePlane)
                        # else:

                        # possible reasons :
                        # 1. the plane is not a face of a convex 3d polygon,
                        #    it is a plane crossing the convex 3d polygon.
                        # 2. the plane is a face of a concave 3d polygon

        # number of faces
        self.__NumberOfFaces = len(faceVerticeIndex)
        # face list
        self.__Faces = []
        # face planes list
        self.__FacePlanes = []

        for i in range(0, self.__NumberOfFaces):

            self.__FacePlanes.append(GeoPlane(fpOutward[i].a, fpOutward[i].b,
                                              fpOutward[i].c, fpOutward[i].d))

            # face vertices
            v = []
            # face vertices indexes
            idx = []

            # number of vertices of the face
            count = len(faceVerticeIndex[i])

            for j in range(0, count):
                idx.append(faceVerticeIndex[i][j])
                v.append(GeoPoint(vertices[idx[j]].x,
                                  vertices[idx[j]].y,
                                  vertices[idx[j]].z))

            self.__Faces.append(GeoFace(v, idx))

    def PointInside3DPolygon(self, x, y, z):

        pt = GeoPoint(x, y, z)

        for i in range(0, self.__NumberOfFaces):
            dis = self.__FacePlanes[i] * pt
            # If the point is in the same half space with normal vector for any face of the cube,
            # then it is outside of the 3D polygon
            if dis > 0:
                return False

        # If the point is in the opposite half space with normal vector for all 6 faces,
        # then it is inside of the 3D polygon
        return True
#-----------------------------------------------------------------------------------------------------------------------


class GeoVector(object):

    def __init__(self, p0, p1):  # GeoPoint p0, p1
        self.__p0 = p0  # read write (get set)
        self.__p1 = p1  # read write (get set)
        self.__x = p1.x - p0.x  # read only
        self.__y = p1.y - p0.y  # read only
        self.__z = p1.z - p0.z  # read only

    @property
    def p0(self):
        return self.__p0

    @p0.setter
    def p0(self, p0):
        self.__p0 = p0
        self.__x = self.p1.x - p0.x
        self.__y = self.p1.y - p0.y
        self.__z = self.p1.z - p0.z

    @property
    def p1(self):
        return self.__p1

    @p1.setter
    def p1(self, p1):
        self.__p1 = p1
        self.__x = p1.x - self.p0.x
        self.__y = p1.y - self.p0.y
        self.__z = p1.z - self.p0.z

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    def __mul__(self, v):  # v: GeoVector
        x = self.y * v.z - self.z * v.y
        y = self.z * v.x - self.x * v.z
        z = self.x * v.y - self.y * v.x
        p0 = v.p0
        p1 = p0 + GeoPoint(x, y, z)
        return GeoVector(p0, p1)
#-----------------------------------------------------------------------------------------------------------------------


class Utility(object):
    @staticmethod
    def ContainsList(listList, listItem):
        listItem.sort()
        for i in range(0, len(listList)):
            temp = listList[i]
            if len(temp) == len(listItem):
                temp.sort()
                itemEqual = True
                for j in range(0, len(listItem)):
                    if temp[j] != listItem[j]:
                        itemEqual = False
                        break
                if itemEqual:
                    return True
            else:
                return False
        return False
