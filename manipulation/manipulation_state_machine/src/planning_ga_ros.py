#!/usr/bin/env python

import rospy

from manipulation_common.msg import PlanFlowerSequenceAction, PlanFlowerSequenceGoal, PlanFlowerSequenceResult

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import kinova_msgs.msg
import actionlib
from manipulation_common.msg import FlowerMap
from manipulation_common.msg import Flower

from tf import TransformListener
import tf.transformations as t_
import numpy as np, random, operator, pandas as pd, matplotlib.pyplot as plt



class Fitness:
    def __init__(self, route):
        self.route = route
        self.distance = 0
        self.fitness= 0.0
    def routeDistance(self):
        if self.distance == 0:
            pathDistance = 0
            for i in range(0, len(self.route)):
                fromFlower = self.route[i]
                toFlower = None
                if i + 1 < len(self.route):
                    toFlower = self.route[i + 1]
                else:
                    toFlower = self.route[0]
                pathDistance += fromFlower.distance(toFlower)
            self.distance = pathDistance
        return self.distance

    def routeFitness(self):
        if self.fitness == 0:
            self.fitness = 1.0 / float(self.routeDistance())
        return self.fitness


class Flower: #Will take flower message as input
    def __init__(self, flower_):
        self.x = flower_.point.point.x
        self.y = flower_.point.point.y
        self.z = flower_.point.point.z
        #self.quaternion = np.copy(np.array([flower_.pose.orientation.x, flower_.pose.orientation.y,flower_.pose.orientation.z,flower_.pose.orientation.w]))
        self.id = flower_.id
        # quaternion distance  = 1 -inner_product(q1,q2)^2
        #quaternion angle = acos (2*inner_product(q1,q2)^2-1)

    def distance(self, flower):
        xDis = abs(self.x - flower.x)
        yDis = abs(self.y - flower.y)
        zDis = abs(self.z - flower.z)
        #print(self.quaternion)
        ##Implement some angle cost
        #print(np.dot(self.quaternion,flower.quaternion))
        #print(1-np.arccos(2*np.dot(self.quaternion,flower.quaternion)**2-1))

        return np.sqrt(np.power(xDis,2) + np.power(xDis,2) +np.power(xDis,2) )
    def __repr__(self):
        return "(" + str(self.id) + ")"


class GA:
    def __init__(self, population, popSize, eliteSize, mutationRate, generations):
        self.pop = self.initialPopulation(popSize, population) # Creates random routes for population
        self.eliteSize = eliteSize
        self.mutationRate = mutationRate
        self.generations = generations
        print("Initial distance: " + str(1 / self.rankRoutes()[0][1]))

        for i in range(0, generations):
            self.nextGeneration()
        print("Final distance: " + str(1 / self.rankRoutes()[0][1]))
        bestRouteIndex = self.rankRoutes()[0][0]
        self.bestRoute = self.pop[bestRouteIndex]
        print ("Best Path: ",self.bestRoute)



    def createRoute(self, flowerList): ##Change for not sampling through all routes ********
        route = random.sample(flowerList, len(flowerList))
        return route

    def initialPopulation(self, popSize, flowerList):
        population = []
        for i in range(0, popSize):
            population.append(self.createRoute(flowerList))
        return population

    def rankRoutes(self):
        fitnessResults = {}
        for i in range(len(self.pop)):
            fitnessResults[i] = Fitness(self.pop[i]).routeFitness()
        return sorted(fitnessResults.items(), key = operator.itemgetter(1), reverse = True)

    def selection(self, ranked):
        selectionResults = []
        df = pd.DataFrame(np.array(ranked), columns=["Index","Fitness"])
        df['cum_sum'] = df.Fitness.cumsum()
        df['cum_perc'] = 100*df.cum_sum/df.Fitness.sum()

        for i in range(self.eliteSize):
            selectionResults.append(ranked[i][0])
        for i in range(len(ranked) - self.eliteSize):
            pick = 100*random.random()
            for i in range(len(ranked)):
                if pick <= df.iat[i,3]:
                    selectionResults.append(ranked[i][0])
                    break
        return selectionResults

    def matingPool(self, selectionResults):
        matingpool = []
        for i in selectionResults:
            matingpool.append(self.pop[i])
        self.pop = matingpool

    def breed(self, parent1, parent2):
        child = []
        childP1 = []
        childP2 = []

        geneA = int(random.random() * len(parent1))
        geneB = int(random.random() * len(parent1))

        startGene = min(geneA, geneB)
        endGene = max(geneA, geneB)

        for i in range(startGene, endGene):
            childP1.append(parent1[i])

        childP2 = [item for item in parent2 if item not in childP1]

        child = childP1 + childP2
        return child


    def breedPopulation(self):
        children = []
        length = len(self.pop) - self.eliteSize
        pool = random.sample(self.pop, len(self.pop))

        for i in range(self.eliteSize):
            children.append(self.pop[i])

        for i in range(length):
            child = self.breed(pool[i], pool[len(self.pop)-i-1])
            children.append(child)

        self.pop = children

    def mutate(self, individual):
        for swapped in range(len(individual)):
            if(random.random() < self.mutationRate):
                swapWith = int(random.random() * len(individual))
                individual[swapped],individual[swapWith] = individual[swapWith],individual[swapped]
        return individual

    def mutatePopulation(self):
        mutatedPop = []
        for ind in range(len(self.pop)):
            mutated_individual_ = self.mutate(self.pop[ind])
            mutatedPop.append(mutated_individual_)
        self.pop = mutatedPop


    def nextGeneration(self):
        popRanked = self.rankRoutes()
        selectionResults = self.selection(popRanked)
        self.matingPool(selectionResults)
        self.breedPopulation()
        self.mutatePopulation()



class Planning_Flower_Sequence:
    def __init__(self):
    	rospy.init_node('planning_flower_sequence', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Planning Node Started ")
        self.tf = TransformListener()
        self.flowers = FlowerMap().map ####CHANGE THE TYPE OF MESSAGE FOR THE SUBSCRIBER ########
        rospy.Subscriber("/flower_mapper/flower_map", FlowerMap, self.global_flower_poses)
        self.server = actionlib.SimpleActionServer('/plan_flower_sequence', PlanFlowerSequenceAction, self.planning_handle, False)
        self.result = PlanFlowerSequenceResult()
        self.pose_array_publisher = rospy.Publisher("/bramblebee/arm/global_flower_poses", PoseArray, queue_size=1)
        self.server.start()
        rospy.spin()


    def global_flower_poses(self,data):
        rospy.loginfo("Flower_read")
        self.flowers = data.map

    def planning_handle(self,goal):
        if self.flowers:
            if len(self.flowers)>1: #Multiple Flowers
                flower_ = self.flower_handle()
                geneticAlgorithm = GA(population=flower_, popSize=100, eliteSize=20, mutationRate=0.01, generations=100)
                self.best_route = geneticAlgorithm.bestRoute
                for i in self.best_route:
                    self.result.sequence.append(i.id)
                self.ee_desired_poses = []
                self.get_desired_previsit_ee_pose()
                self.result.ee_previsit_poses = self.ee_desired_poses
                self.server.set_succeeded(self.result)
                self.result = PlanFlowerSequenceResult()
                rospy.loginfo("Planning Finished")

            else: #Single Flower
                rospy.loginfo("single flower")
                self.result.sequence = [self.flowers[0].id]
                self.get_desired_single_flower_pose()
                self.result.ee_previsit_poses = [self.ee_pose]
                self.server.set_succeeded(self.result)
                self.result = PlanFlowerSequenceResult()
                rospy.loginfo("Planning Finished")
                rospy.sleep(3)

        else: #No Flower
            rospy.loginfo("No Flowers Detected to plan")
            rospy.sleep(3)
            self.server.set_succeeded(self.result)

    def flower_handle(self):
        flower_ = []
        for i in range(len(self.flowers)):
            flower_.append(Flower(self.flowers[i]))
        return flower_

    def get_desired_single_flower_pose(self): # Rewrite and put together with get_desired_previsit_ee_pose
        self.ee_pose = Pose()
        flower_ = self.flowers[0]
        pos_flower = np.array([flower_.point.point.x, flower_.point.point.y, flower_.point.point.z])
        new_offset = np.multiply(np.array([-0.05,-0.05,-0.05]),np.array([flower_.vec.vector.x,flower_.vec.vector.y,flower_.vec.vector.z]))
        #rot_flower = t_.quaternion_matrix([flower_.pose.orientation.x,
        #                                    flower_.pose.orientation.y,
        #                                    flower_.pose.orientation.z,
        #                                    flower_.pose.orientation.w])
        #rot = np.array ([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        #des_rot = np.matmul(np.copy(rot),np.copy(np.transpose(rot_flower)))
        #des_quat = t_.quaternion_from_matrix(des_rot)
    #    offset = np.multiply(np.array([-0.2,-0.2,-0.2]),np.array([des_rot[2][0],des_rot[2][1],des_rot[2][2]]))
        des_pos = pos_flower+new_offset
        self.ee_pose.position.x= des_pos[0]
        self.ee_pose.position.y= des_pos[1]
        self.ee_pose.position.z= des_pos[2]

        #Follow Jared Instructions
        n_ = np.multiply(np.array([flower_.vec.vector.x, flower_.vec.vector.y, flower_.vec.vector.z]),-1)
        print("n_vector:",n_)
        _,quat = self.get_transform_end_effector_to_global()
        gRe = t_.quaternion_matrix(quat)[0:3,0:3]
        print("gRe:",gRe)
        q_ = np.multiply(gRe,np.transpose([0,0,1]))[0:3,2]
        print("q_vector:",q_)
        u_ = np.multiply(np.dot(np.copy(n_),np.copy(n_)),np.copy(n_))/np.linalg.norm(np.multiply(np.dot(np.copy(n_),np.copy(n_)),np.copy(n_)))
        print("u_:",u_)
        v_ = (np.copy(q_)-np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_)))/np.linalg.norm((np.copy(q_)-np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_))))
        print("v_:",v_)
        w_ = np.cross(np.copy(q_),np.copy(n_))/np.linalg.norm(np.cross(np.copy(q_),np.copy(n_)))
        F = np.concatenate(([np.transpose(u_)],[np.transpose(v_)],[np.transpose(w_)]))  #[u,v,w]^-1
        print("F_:",F)
        G = np.array([[np.dot(n_,q_),-1*np.linalg.norm(np.cross(q_,n_)),0],[np.linalg.norm(np.cross(q_,n_)),np.dot(n_,q_),0],[0,0,1]])
        eRe_prime=np.multiply(np.transpose(F),np.multiply(G,F))
        gRe_prime=np.multiply(np.copy(gRe),np.copy(eRe_prime))
        print("gRe_new': ",gRe_prime)
        homog_matrixes = np.concatenate((np.concatenate((gRe_prime,np.transpose([[0,0,0]])),axis=1),np.array([[0,0,0,1]])))
        des_quat = t_.quaternion_from_matrix(homog_matrixes)
        print(des_quat)
        self.ee_pose.orientation.x= des_quat[0]
        self.ee_pose.orientation.y= des_quat[1]
        self.ee_pose.orientation.z= des_quat[2]
        self.ee_pose.orientation.w= des_quat[3]

    def get_desired_previsit_ee_pose(self):
        for i in self.best_route:
            for flower_ in self.flowers:
                if flower_.id == i.id:
                    self.ee_pose = Pose()
                    pos_flower = np.array([flower_.point.point.x, flower_.point.point.y, flower_.point.point.z])
                    new_offset = np.multiply(np.array([-0.05,-0.05,-0.05]),np.array([flower_.vec.vector.x,flower_.vec.vector.y,flower_.vec.vector.z]))
                    #rot_flower = t_.quaternion_matrix([flower_.pose.orientation.x,
                    #                                    flower_.pose.orientation.y,
                    #                                    flower_.pose.orientation.z,
                    #                                    flower_.pose.orientation.w])
                    #rot = np.array ([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
                    #des_rot = np.matmul(np.copy(rot),np.copy(np.transpose(rot_flower)))
                    #des_quat = t_.quaternion_from_matrix(des_rot)
#                    offset = np.multiply(np.array([-0.2,-0.2,-0.2]),np.array([des_rot[2][0],des_rot[2][1],des_rot[2][2]]))
                    print(" ")
                    print("Flower topic:",flower_)
                    des_pos = np.copy(pos_flower)+np.copy(new_offset)
                    print("Desired Pose", des_pos)

                    self.ee_pose.position.x= des_pos[0]
                    self.ee_pose.position.y= des_pos[1]
                    self.ee_pose.position.z= des_pos[2]

                    #Follow Jared Instructions
                    n_ = np.multiply(np.array([flower_.vec.vector.x, flower_.vec.vector.y, flower_.vec.vector.z]),-1)
                    print("n_vector:",n_)
                    _,quat = self.get_transform_end_effector_to_global()
                    gRe = t_.quaternion_matrix(quat)[0:3,0:3]
                    print("gRe:",gRe)
                    q_ = np.multiply(gRe,np.transpose([0,0,1]))[0:3,2]
                    print("q_vector:",q_)

                    u_ = np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_))/np.linalg.norm(np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_)))
                    print("u_:",u_)
                    v_ = (np.copy(q_)-np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_)))/np.linalg.norm((np.copy(q_)-np.multiply(np.dot(np.copy(n_),np.copy(q_)),np.copy(n_))))
                    print("v_:",v_)
                    w_ = np.cross(np.copy(q_),np.copy(n_))/np.linalg.norm(np.cross(np.copy(q_),np.copy(n_)))
                    print("w_:",w_)
                    print("u v w normal:", np.linalg.norm(u_),np.linalg.norm(v_),np.linalg.norm(w_))

                    F = (np.concatenate(([np.transpose(u_)],[np.transpose(v_)],[np.transpose(w_)])))  #[u,v,w]^-1
                    print("F_:",F)


                    G = np.array([[np.dot(n_,q_),-1*np.linalg.norm(np.cross(q_,n_)),0],[np.linalg.norm(np.cross(q_,n_)),np.dot(n_,q_),0],[0,0,1]])
                    print("G", G)
                    if n_[2]>0:
                        temp1_ = np.matmul(np.copy(np.linalg.inv(G)),np.copy(F))
                        print("Temp(g is inverse)",temp1_)
                        eRe_prime=np.matmul(np.linalg.inv(F),temp1_)
                        print("eRe_prime",eRe_prime)
                    else:
                        temp1_ = np.matmul(np.copy(G),np.copy(F))
                        print("Temp",temp1_)
                        eRe_prime=np.matmul(np.linalg.inv(F),temp1_)
                        print("eRe_prime",eRe_prime)
                    gRe_prime=np.matmul(np.copy(eRe_prime),np.copy(gRe))
                    print("gRe_prime': ",gRe_prime)
                    print("gRe_prime_Normal", np.linalg.norm(gRe_prime[:,0]),np.linalg.norm(gRe_prime[:,1]),np.linalg.norm(gRe_prime[:,2]))
                    homog_matrixes = np.concatenate((np.concatenate((gRe_prime,np.transpose([[0,0,0]])),axis=1),np.array([[0,0,0,1]])))
                    print("homogeneous_matrix",homog_matrixes)
                    des_quat = t_.quaternion_from_matrix(homog_matrixes)
                    print("Quaternion", des_quat)

                    self.ee_pose.orientation.x= des_quat[0]
                    self.ee_pose.orientation.y= des_quat[1]
                    self.ee_pose.orientation.z= des_quat[2]
                    self.ee_pose.orientation.w= des_quat[3]
                    self.ee_desired_poses.append(self.ee_pose)
                    print(self.ee_pose)
        self.test_pose = rospy.Publisher("/test_pose", PoseArray, queue_size=1)
        #fix issue with publishing before connection established
        while self.test_pose.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscriber to connect")
            rospy.sleep(1)
        self.test_pose_message = PoseArray()
        self.test_pose_message.header.frame_id = "j2n6s300_link_base"
        self.test_pose_message.poses = self.ee_desired_poses
        print("TEST POSE: ",self.test_pose_message)
        self.test_pose.publish(self.test_pose_message)
        rospy.sleep(1)

    def get_transform_end_effector_to_global(self):
#        if self.tf.frameExists("/j2n6s300_link_base") and self.tf.frameExists("/j2n6s300_end_effector"):
        t = self.tf.getLatestCommonTime("/j2n6s300_end_effector", "/j2n6s300_link_base")
        position, quaternion = self.tf.lookupTransform("/j2n6s300_end_effector", "/j2n6s300_link_base", t)
        print(quaternion)
        return position, quaternion
    def shutdown(self):
        rospy.loginfo("Planning node is shutdown")
        rospy.sleep(1)


def main():
    try:
        planninf = Planning_Flower_Sequence()

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()
