from general import pm, parentHierarchically, getBoundingBoxSize, \
    getFlattenList, RigGroups, Controllers


class Car:
    def __init__(self):
        self.sizeRatio = 1
        self.sizeCtrl = "cc_sizeController"
        self.mainCtrl = "cc_main"
        self.subCtrl = "cc_sub"
        self.rootJoint = "jnt_root"
        self.jntNameAndPos = {
            "jnt_root": (0, 15, 0), 
            "jnt_body": (0, 45, 0), 
            "jnt_bodyEnd": (0, 145, 0), 
            "jnt_wheelLeftFront": (70, 30, 140), 
            "jnt_wheelLeftFrontEnd": (85, 30, 140), 
            "jnt_wheelRightFront": (-70, 30, 140), 
            "jnt_wheelRightFrontEnd": (-85, 30, 140), 
            "jnt_wheelLeftRear": (70, 30, -140), 
            "jnt_wheelLeftRearEnd": (85, 30, -140), 
            "jnt_wheelRightRear": (-70, 30, -140), 
            "jnt_wheelRightRearEnd": (-85, 30, -140), 
        }
        self.hierarchy = {
            "jnt_root": [
                [f"jnt_body{i}" for i in ["", "End"]], 
                [f"jnt_wheelLeftFront{i}" for i in ["", "End"]], 
                [f"jnt_wheelRightFront{i}" for i in ["", "End"]], 
                [f"jnt_wheelLeftRear{i}" for i in ["", "End"]], 
                [f"jnt_wheelRightRear{i}" for i in ["", "End"]], 
                ], 
            }


    def createJoints(self):
        # CleanUp first
        self.cleanUp()
        # Create joints
        for jnt, pos in self.jntNameAndPos.items():
            pm.select(cl=True)
            pm.joint(p=pos, n=jnt)
        # Set hierarchy
        for parents, childList in self.hierarchy.items():
            for children in childList:
                parentHierarchically(children)
                pm.makeIdentity(children, a=1, t=1, r=1, s=1, jo=1)
                pm.parent(children[0], parents)
        # Create a controller
        self.sizeRatio = getBoundingBoxSize(self.rootJoint)
        pm.circle(nr=(0,1,0), ch=0, n=self.sizeCtrl, r=self.sizeRatio)
        pm.parent(self.rootJoint, self.sizeCtrl)


    def cleanUp(self):
        delGroups = [self.sizeCtrl]
        joints = list(self.jntNameAndPos.keys())
        grpNames = list(RigGroups().groupNames.keys())[1:]
        grpNames += getFlattenList(list(RigGroups().groupNames.values())[1:])
        delGroups += joints + grpNames
        for i in delGroups:
            try:
                pm.delete(i)
            except:
                continue


    def updatePosition(self):
        for jnt in self.jntNameAndPos.keys():
            pos = pm.xform(jnt, q=True, t=True, ws=True)
            self.jntNameAndPos[jnt] = tuple(pos)


    def sameBothSide(self, side: str="LeftToRight"):
        """ The default change is from left to right. 
        But the opposite is also possible.
        >>> sameBothSide()
        >>> sameBothSide("RightToLeft")
         """
        # Update first
        self.updatePosition()
        # Split both side
        A, B = side.split("To")
        aSide = []
        bSide = []
        for jntName in self.jntNameAndPos.keys():
            if A in jntName:
                aSide.append(jntName)
            elif B in jntName:
                bSide.append(jntName)
            else:
                continue
        # Update opposite
        for idx, aJoint in enumerate(aSide):
            x, y, z = pm.xform(aJoint, q=True, t=True, ws=True)
            bJoint = bSide[idx]
            self.jntNameAndPos[bJoint] = (-1*x, y, z)
        # Create joints again
        self.createJoints()


    def build(self):
        ctrl = Controllers()
        cc_name = {"car": "cc_body", "car2": "cc_sub", "car3": "cc_main"}
        ctrl.createControllers(**cc_name)
        print(self.sizeRatio)



qcCar = Car()
# qcCar.createJoints()
# qcCar.cleanUp()
# qcCar.sameBothSide()
qcCar.build()


