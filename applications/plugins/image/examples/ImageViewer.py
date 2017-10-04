def createScene(node):
    node.createObject('RequiredPlugin', pluginName="image", name="imagePlugin")
    node.createObject('BackgroundSetting',color='1 1 1')
    node.createObject('ImageContainer', name="image", filename="textures/cubemap_bk.bmp", transform="-5 -5 0 0 0 0 0.1 0.1 30 0 1 1")
    imv = node.createObject('ImageViewer',  name="viewer", src="@image")
    imv.init()
    imv.bwdInit()

    print imv.getPlaneQuads()
    sys.stdout.flush()

