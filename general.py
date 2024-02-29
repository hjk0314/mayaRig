from collections import Iterable
import pymel.core as pm


def getPosition(selection: str) -> list:
    """ Get the coordinates of an object or point.
    >>> getPosition("pSphere1")
    >>> getPosition("pSphere1.vtx[317]")
    >>> [0.0, 0.0, 0.0]
     """
    try:
        position = pm.pointPosition(selection)
    except:
        position = pm.xform(selection, q=1, ws=1, rp=1)
    return position


def makeSameAsParentPivot(object, parents) -> None:
    """ If you put object under parents and freeze it, 
    the pivots match together. """
    parentsPivot = pm.xform(parents, q=1, ws=1, rp=1)
    pm.xform(object, sp=parentsPivot, rp=parentsPivot)
    pm.parent(object, parents)
    pm.makeIdentity(object, a=1, t=1, r=1, s=1, n=0, pn=1)
    pm.parent(object, w=True)


def getFlattenList(*args):
    """ Flattens a list within a list. 
    >>> args = (["ab", ["bc"], ["ef"]], [[["gh", ], "ij"], "jk"],)
    >>> return ['ab', 'bc', 'ef', 'gh', 'ij', 'jk']
     """
    result = []
    for arg in args:
        if not isinstance(arg, str) and isinstance(arg, Iterable):
            for i in arg:
                result.extend(getFlattenList(i))
        else:
            result.append(arg)
    return result


def getBoundingBoxPosition(vertexOrObject) -> list:
    boundingBox = pm.xform(vertexOrObject, q=True, bb=True, ws=True)
    xMin, yMin, zMin, xMax, yMax, zMax = boundingBox
    x = (xMin + xMax) / 2
    y = (yMin + yMax) / 2
    z = (zMin + zMax) / 2
    return [x, y, z]


def orientJoints(joints=[], primaryAxis='yzx', secondaryAxis='zup'):
    """ The default value of primaryAxis and secondaryAxis are 
    the same as Mixamo spine. """
    if joints:
        allJoints = [pm.PyNode(i) for i in joints]
    else:
        allJoints = pm.ls(sl=True)
    endJoints = [i for i in allJoints if not i.getChildren()]
    initJoint = allJoints[0]
    pm.makeIdentity(allJoints, a=True, jo=True, n=0)
    pm.joint(initJoint, 
            e=True, # edit
            oj=primaryAxis, # orientJoint
            sao=secondaryAxis, # secondaryAxisOrient
            ch=True, # children
            zso=True, # zeroScaleOrient
        )
    for i in endJoints:
        pm.joint(i, e=True, oj='none', ch=True, zso=True)


def createCurvePassingKeyedUp(startFrame, endFrame, objects=[]):
    sel = objects if objects else pm.ls(sl=True)
    curves = []
    for i in sel:
        positions = []
        for frame in range(startFrame, endFrame + 1):
            pm.currentTime(frame)
            pos = getPosition(i)
            positions.append(pos)
        cuv = pm.curve(p=positions, d=3)
        curves.append(cuv)
    return curves


def createCurvePassingThrough(objects=[]) -> str:
    """ Return curveName """
    sel = objects if objects else pm.ls(sl=True, fl=True)
    positions = [getPosition(i) for i in sel]
    curve = pm.curve(ep=positions, d=3)
    return curve


def createClosedCurve(objects=[]) -> str:
    """ The closedCurve means that 
    the start and end points of a curve are connected.
    >>> createCurvePassingLocators()
    >>> createCurvePassingLocators(curveClosed=True)
    >>> return "circleName"
     """
    sel = objects if objects else pm.ls(sl=True, fl=True)
    positions = [getPosition(i) for i in sel]
    circle = pm.circle(nr=(0, 1, 0), ch=False, s=len(sel))
    circle = circle[0]
    for i, pos in enumerate(positions):
        pm.move(f"{circle}.cv[{i}]", pos, ws=True)
    return circle


def createCurveAimingPoint(objects=[]) -> str:
    """ Select two objects or points.
    A straight line is created looking at the last point.
    >>> return "curveName"
     """
    sel = objects if objects else pm.ls(sl=True, fl=True)
    positions = [getPosition(i) for i in [sel[0], sel[-1]]]
    simpleCurve = pm.curve(p=positions, d=1)
    locators = []
    for i in positions:
        locator = pm.spaceLocator()
        pm.move(locator, i)
        locators.append(locator)
    startLocator, endLocator = locators
    pm.aimConstraint(endLocator, startLocator)
    pm.delete(startLocator, cn=True)
    makeSameAsParentPivot(simpleCurve, startLocator)
    pm.rebuildCurve(simpleCurve, d=3, ch=0, s=3, rpo=1, end=1, kr=0, kt=0)
    pm.delete(locators)
    return simpleCurve


def createCurvesNormalDirection(vertex=[]) -> list:
    sel = vertex if vertex else pm.ls(sl=True, fl=True)
    result = []
    for vtx in sel:
        vertexPosition = pm.pointPosition(vtx)
        normalVector = pm.polyNormalPerVertex(vtx, q=True, normalXYZ=True)
        normalVector = normalVector[0:3]
        locators = []
        for pos in [(0, 0, 0), normalVector]:
            locator = pm.spaceLocator()
            locators.append(locator)
            pm.move(locator, pos)
        unitCurve = createCurveAimingPoint(locators)
        pm.move(unitCurve, vertexPosition)
        pm.delete(locators)
        result.append(unitCurve)
    return result



