import os
import math
import sys
import numpy as np
import Compliant.types as Types
import SofaPython.Quaternion
import SofaPython.SofaNumpy as SofaNumpy

def loadOBJ(filename):
    # Check if file exist
    if filename == "" or not filename: return

    filename = os.path.realpath(filename)
    if not os.path.exists(filename):
        return

    # Output data
    vertices = []
    normals = []
    texcoords = []
    faces = []

    # Init the append for everything
    vertices_append = vertices.append
    normals_append = normals.append
    texcoords_append = texcoords.append
    faces_append = faces.append

    # Begin loop
    file_content = open(filename, "r")
    for line in file_content:
        if line.startswith('#'):
            continue
        values = line.split()
        if not values:
            continue
        if values[0] == 'v':
            vertices_append(map(float, values[1:4]))
        elif values[0] == 'vn':
            normals_append(map(float, values[1:4]))
        elif values[0] == 'vt':
            texcoords_append(map(float, values[1:3]))
        elif values[0] == 'f':
            face = list(); tcoords = list(); norms = list()
            f_append = face.append; t_append = tcoords.append; n_append = norms.append
            for v in values[1:]:
                w = v.split('/')
                f_append(int(w[0])-1)
                if len(w) > 1 and len(w[1]) > 0:
                    t_append(int(w[1])-1)
                else:
                    t_append(0)
                if len(w) > 2 and len(w[2]) > 0:
                    n_append(int(w[2])-1)
                else:
                    n_append(0)
            faces_append((face, norms, tcoords))
    return (vertices, normals, texcoords, faces)


"""
Helper class for manipulating armature with bone hierarchy exported in sml
"""
class Armature:

    def __init__(self, model, scene):
        self.scene = scene
        self.model = model
        self.children = {}
        self.localCoordinates = {}
        self.initialLocalCoordinates = {}
        for k,solid in self.model.solids.iteritems():
            if solid.parent:
                if solid.parent in self.children:
                    self.children[solid.parent].append(solid.id)
                else:
                    self.children[solid.parent] = [solid.id]

                position = np.array(self.scene.rigidScales[solid.id].rigidDofs.position).view(dtype=Types.Rigid3)[0]
                parentPosition = np.array(self.scene.rigidScales[solid.parent].rigidDofs.position).view(dtype=Types.Rigid3)[0]
                self.localCoordinates[solid.id] = parentPosition.inv() * position   
                self.initialLocalCoordinates[solid.id] = self.localCoordinates[solid.id]


    def updateChildren(self, bone):
        if(bone in self.children):
            position = SofaNumpy.numpy_data(self.scene.rigidScales[bone].rigidDofs,"position").view(dtype=Types.Rigid3)[0]
            for child in self.children[bone]:
                newPosition = position*self.localCoordinates[child]
                SofaNumpy.numpy_data(self.scene.rigidScales[child].rigidDofs,"position").view(dtype=Types.Rigid3)[0] = newPosition
                self.updateChildren(child)
                
        return


    def translate(self, bone, translation):
        parent = self.model.solids[bone].parent
        if(parent):
            parentPosition = np.array(self.scene.rigidScales[self.model.solids[bone].parent].rigidDofs.position).view(dtype=Types.Rigid3)[0]
            initPosition = parentPosition * self.initialLocalCoordinates[bone];
        else:
            initPosition = np.array(self.scene.rigidScales[bone].rigidDofs.reset_position).view(dtype=Types.Rigid3)[0]

        position = SofaNumpy.numpy_data(self.scene.rigidScales[bone].rigidDofs,"position").view(dtype=Types.Rigid3)[0]
        position.center = initPosition.center + translation
        self.updateChildren(bone)
        if(self.model.solids[bone].parent):
            
            self.localCoordinates[bone] = parentPosition.inv() * position      
        return


    def rotate(self, bone, rotation):
        parent = self.model.solids[bone].parent
        if(parent):
            parentPosition = np.array(self.scene.rigidScales[self.model.solids[bone].parent].rigidDofs.position).view(dtype=Types.Rigid3)[0]
            initPosition = parentPosition * self.initialLocalCoordinates[bone];
        else:
            initPosition = np.array(self.scene.rigidScales[bone].rigidDofs.reset_position).view(dtype=Types.Rigid3)[0]

        position = SofaNumpy.numpy_data(self.scene.rigidScales[bone].rigidDofs,"position").view(dtype=Types.Rigid3)[0]
        q = SofaPython.Quaternion.from_euler(rotation[:]).view(dtype=Types.Quaternion)
        position.orient = initPosition.orient * q
        self.updateChildren(bone)
        if(parent):
            self.localCoordinates[bone] = parentPosition.inv() * position      


    def scale(self, bone, scale):
        self.scene.rigidScales[bone].scaleDofs.position = scale