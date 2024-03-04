from general import *


class Car:
    def __init__(self):
        self.mainCtrl = "cc_main"
        self.subCtrl = "cc_sub"
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
        self.hierarchy1 = {
            "center": ["jnt_root", "jnt_body", "jnt_bodyEnd"], 
            "LeftFront": [f"jnt_wheelLeftFront{i}" for i in ["", "End"]], 
            "RightFront": [f"jnt_wheelRightFront{i}" for i in ["", "End"]], 
            "LeftRear": [f"jnt_wheelLeftRear{i}" for i in ["", "End"]], 
            "RightRear": [f"jnt_wheelRightRear{i}" for i in ["", "End"]], 
        }


    def setupUI(self):
        """ winName = 'QuickRig_CAR'
        if pm.window(winName, exists=True):
            pm.deleteUI(winName)
        win = pm.window(winName, t='Car Rig', s=True, rtf=True)
        pm.columnLayout(cat=('both', 4), rs=2, cw=178)
        pm.separator(h=10)
        self.txt1 = pm.textField(ed=True)
        pm.button(l="Create Temp Joint", c=lambda x: self.createTempJoints())
        pm.button(l="Create controllers", c=lambda x: self.createCtrls())
        pm.button(l="Get Radius", c=lambda x: self.displayRadius())
        pm.button(l="Close", c=lambda x: pm.deleteUI(winName))
        pm.separator(h=10)
        pm.showWindow(win) """
        pass


    def createJoints(self):
        try: pm.delete(self.circle)
        except: pass
        carJnt = self.point.values()
        branch = []
        for i in carJnt:
            pm.select(cl=True)
            for name, pos in i.items():
                jnt = pm.joint(p=(0, 0, 0), n=name, rad=10)
                pm.move(jnt, pos)
            headJnt = [j for j in i.keys()][0]
            branch.append(headJnt)
        hjk.orientJnt(branch)
        firstJnt = branch.pop(0)
        self.circle = hjk.createCircle(self.circle, 300, y=True)
        pm.parent(branch, firstJnt)
        pm.parent(firstJnt, self.circle)


    def createCtrls(self):
        ccMain = self.createMainCtrl()
        ccWheel = self.createWheelCtrl()
        self.createGroup(ccMain, ccWheel)
        hjk.AutoWheel_Rig(ccWheel)
        self.finish(ccMain, ccWheel)


    def createMainCtrl(self):
        ccDict = {
            "cc_main": {"car3": True}, 
            "cc_sub": {"car2": True}, 
            "cc_body": {"car": True}, 
        }
        carCircleSize = pm.getAttr(f"{self.circle}.scale")
        result = []
        for ccName, ctrlName in ccDict.items():
            tmp = hjk.ctrl(ctrlName)[0]
            cuv = pm.rename(tmp, ccName)
            pm.scale(cuv, carCircleSize)
            pm.makeIdentity(cuv, t=0, r=0, s=1, n=0, pn=0, a=True)
            if ccName == "cc_body":
                jnt = ccName.replace("cc_", "jnt_")
                pm.matchTransform(ccName, jnt, pos=True)
            result.append(cuv)
        return result


    def createWheelCtrl(self):
        carCircleSize = pm.getAttr(f"{self.circle}.scale")
        s = max(carCircleSize)
        jntWheelList = self.makeWheelList()
        result = []
        for i in jntWheelList:
            cc = i.replace("jnt_", "cc_")
            cc = pm.circle(nr=(1, 0, 0), n=cc, r=40, ch=0)[0]
            pm.scale(cc, [s, s, s])
            pm.makeIdentity(cc, t=0, r=0, s=1, n=0, pn=0, a=True)
            pm.matchTransform(cc, i, pos=True)
            result.append(cc)
        return result


    def makeWheelList(self):
        pm.select(self.circle, hi=True)
        selAll = pm.ls(sl=True)
        A = "wheel"
        B = "end"
        result = []
        for i in selAll:
            tmp = i.split("_")
            if A in tmp and B not in tmp:
                result.append(i)
            else:
                continue
        return result


    def createGroup(self, ccMain: list, ccWheel: list):
        # Create groups with grpList.
        grpCtrl = [pm.group(em=True, n=i) for i in self.grpList].pop()
        # Create cc_main groups.
        grpMain = [hjk.groupingEmpty(i)[0] for i in ccMain]
        init = grpMain.pop(0)
        # body = ccMain.pop(-1)
        for idx in range(2):
            pm.parent(grpMain[idx], ccMain[idx])
        pm.parent(init, grpCtrl)
        # Create cc_wheel groups.
        grpWheel = [hjk.groupingEmpty(i)[0] for i in ccWheel]
        pm.parent(grpWheel, grpCtrl)


    def finish(self, ccMain, ccWheel):
        main, sub, body = ccMain
        yellowList = [main, body]
        pinkList = [sub]
        redList = [ccWheel[i] for i in range(0, 4, 2)]
        blueList = [ccWheel[i] for i in range(1, 4, 2)]
        colorDict = {
            "yellow": yellowList, 
            "pink": pinkList, 
            "red": redList, 
            "blue": blueList, 
        }
        for i in ccWheel:
            offset = i.getParent()
            pm.parentConstraint(sub, offset, mo=True, w=1)
            pm.scaleConstraint(sub, offset, mo=True, w=1)
        hjk.Colors.colors(colorDict)
        pm.delete(self.circle)


    def displayRadius(self):
        sel = pm.ls(sl=True)
        if not sel:
            result = "Select wheel object."
        else:
            sizeUp = 1
            wheels = {i: sizeUp for i in sel}
            radius = self.getRadius(wheels)
            result = [str(i) for i in radius]
            result = str(" ".join(result))
        self.txt1.setText(result)


    def getRadius(self, obj_sizeUp: dict):
        """ Create the controller 1.2 times the sizeUp of the object.
        If no parameters are given, the selected object is used.
         """
        result = []
        for obj, sizeUp in obj_sizeUp.items():
            bBox = pm.xform(obj, q=True, bb=True)
            xMin, yMin, zMin, xMax, yMax, zMax = bBox
            x = (xMax - xMin) / 2
            y = (yMax - yMin) / 2
            z = (zMax - zMin) / 2
            radius = max([x, y, z])
            radius = round(radius*sizeUp, 3)
            result.append(radius)
        return result

