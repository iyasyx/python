import networkx as nx
import random
import csv
import os
import matplotlib.pyplot as plt

# ------------------------------ Configuration Parameters ------------------------------
NUM_ATM = random.randint(5, 15)  # Number of Cash Machines (nodes)
MINDISTANCE = 1  # Minimum distance between Machines
MAXDISTANCE = 20  # Maximum distance between Machines
MIN_NEIGHBORS = 2  # Minimum number of neighbors per Machine
MAX_NEIGHBORS = 4  # Maximum number of neighbors per Machine
TIME_LIMIT = 24  # Maximum time for robbery (hours)
ROBBERY_SPEED = 100  # Robbery speed (km/h)
ROBBERY_TIME_PER_KM = 1 / ROBBERY_SPEED  # Hours per km
ROBBERY_SERVICE_TIME = 1  # Time to rob each ATM (hours)
ROBBERY_START_TIME = 3  # Time at which robbers start (hours)
TIME_PER_KM = 0.05  # Time per km for service (1 km in 60 km/h)
SERVICE_TIME = 0.5  # Time to refill each ATM (30 minutes)
SAVE_PATH = "/Users/yasamansmacbook/Documents/AD_project/csv"  # Path to save CSV files

# ------------------------------ Classes ------------------------------
class CashMachine:
    def __init__(self, id, is_priority=False, is_start=False):
        self.id = id
        self.is_priority = is_priority
        self.is_start = is_start

    def __str__(self):
        return self.id

class Network:
    def __init__(self):
        self.machines = []  # list of CashMachine objects
        self.connections = {}  # dictionary of edges with weights

    def addMachine(self, machine):
        if machine not in self.machines:
            self.machines.append(machine)
            self.connections[machine] = []

    def addConnection(self, machine1, machine2, distance):
        if machine1 in self.machines and machine2 in self.machines:
            self.connections[machine1].append((machine2, distance))
            self.connections[machine2].append((machine1, distance))  # for undirected graph

    def toNetworkx(self):
        # converts the network into a NetworkX graph for visualization
        G = nx.Graph()

        for machine in self.machines:
            G.add_node(str(machine), is_priority=machine.is_priority, is_start=machine.is_start)

        for machine, connections in self.connections.items():
            for connectedMachine, distance in connections:
                G.add_edge(str(machine), str(connectedMachine), weight=distance)
        return G

# -------------------------- Utility Functions --------------------------
def createNetwork(numMachines=NUM_ATM, mindistance=MINDISTANCE, maxdistance=MAXDISTANCE, minNeighbors=MIN_NEIGHBORS, maxNeighbors=MAX_NEIGHBORS):
    network = Network()

    for i in range(numMachines):
        network.addMachine(CashMachine(f"{i + 1}"))

    for machine1 in network.machines:
        possibleNeighbors = [
            machine2
            for machine2 in network.machines
            if machine2 != machine1
            and machine2 not in [neighbor[0] for neighbor in network.connections[machine1]]
        ]

        if len(possibleNeighbors) > 0:
            # Calculate the valid range for the number of neighbors
            minVal = max(0, minNeighbors - len(network.connections[machine1]))
            maxVal = min(len(possibleNeighbors), maxNeighbors - len(network.connections[machine1]))

            # Ensure maxVal is greater than minVal, otherwise, skip adding connections
            if minVal < maxVal:
                neighbors = random.sample(possibleNeighbors, random.randint(minVal, maxVal))
                for machine2 in neighbors:
                    distance = random.randint(mindistance, maxdistance)
                    network.addConnection(machine1, machine2, distance)

    return network

def saveToCSV(fileName, data, headers, savePath=SAVE_PATH):
    if not os.path.exists(savePath):
        os.makedirs(savePath)

    filePath = os.path.join(savePath, fileName)

    with open(filePath, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(data)

def displayNetwork(network):
    nxNetwork = network.toNetworkx()
    positions = nx.spring_layout(nxNetwork, k=5, iterations=500)

    colors = ["red" if nodeData["is_priority"] else "blue" for _, nodeData in nxNetwork.nodes(data=True)]
    nx.draw(nxNetwork, positions, with_labels=True, node_color=colors, node_size=1500, font_size=12)

    startNode = next(node for node, data in nxNetwork.nodes(data=True) if data["is_start"])
    startColor = "red" if nxNetwork.nodes[startNode]["is_priority"] else "blue"
    nx.draw_networkx_nodes(nxNetwork, positions, nodelist=[startNode], node_size=1500, node_color=startColor, edgecolors="black", linewidths=1.3)

    nx.draw_networkx_edge_labels(nxNetwork, positions, edge_labels=nx.get_edge_attributes(nxNetwork, "weight"), font_size=8)

    plt.title("Network Visualization")
    plt.show()

# -------------------------- Process Simulation --------------------------
def calculateShortestPath(network, startMachine):
    distances = {machine: float('inf') for machine in network.machines}
    previousMachines = {machine: None for machine in network.machines}
    distances[startMachine] = 0

    machines = set(network.machines)

    while machines:
        currentMachine = min(machines, key=lambda machine: distances[machine])
        machines.remove(currentMachine)

        for neighbor, distance in network.connections[currentMachine]:
            alternativeRoute = distances[currentMachine] + distance
            if alternativeRoute < distances[neighbor]:
                distances[neighbor] = alternativeRoute
                previousMachines[neighbor] = currentMachine

    return distances, previousMachines

def findPath(previousMachines, targetMachine):
    path = []
    currentMachine = targetMachine

    while currentMachine is not None:
        path.insert(0, currentMachine)
        currentMachine = previousMachines[currentMachine]

    return path

def robberySimulation(network, startMachine, timeLimit=TIME_LIMIT, savePath=SAVE_PATH):
    # Step 1: Initialize robbery parameters
    robberySpeed = ROBBERY_SPEED  # km/h
    robberyTimePerKm = ROBBERY_TIME_PER_KM  # hours per km
    robberyServiceTime = ROBBERY_SERVICE_TIME  # 1 hour per robbery
    robberyStartTime = ROBBERY_START_TIME  # hours
    currentTime = robberyStartTime

    # Step 2: Select random starting point for robbers
    unvisitedMachines = [machine for machine in network.machines if not machine.is_start]
    random.shuffle(unvisitedMachines)
    robbersStartMachine = random.choice(unvisitedMachines)

    # Initialize robbery tracking
    robbedMachines = []
    currentMachine = robbersStartMachine
    totalRobberyTime = 0

    while currentTime < timeLimit and unvisitedMachines:
        # Calculate shortest paths from current machine
        distances, previousMachines = calculateShortestPath(network, currentMachine)

        # Find next machine to rob with minimal distance
        nextMachine = min(unvisitedMachines, key=lambda machine: distances[machine])

        # Calculate travel time and check if it fits within the time limit
        travelTime = distances[nextMachine] * robberyTimePerKm
        if currentTime + travelTime + robberyServiceTime > timeLimit:
            break

        # Update time and record robbery
        currentTime += travelTime + robberyServiceTime
        totalRobberyTime += travelTime + robberyServiceTime
        path = findPath(previousMachines, nextMachine)
        robbedMachines.append({
            "Machine": nextMachine.id,
            "Path": [machine.id for machine in path],
            "Time": round(currentTime, 2),
            "Distance": distances[nextMachine],
        })

        # Remove robbed machine from unvisited list and update current location
        unvisitedMachines.remove(nextMachine)
        currentMachine = nextMachine

    # Step 3: Calculate statistics and save data
    avgRobberyTime = totalRobberyTime / len(robbedMachines) if robbedMachines else 0

    print("\n----- Robbery Simulation -----")
    print("\nRobbery Events:")
    for machine in robbedMachines:
        print(f"Robbery occurred at Machine {machine['Machine']}, via path {machine['Path']} at time: {machine['Time']} and distance: {machine['Distance']}")

    print("\nList of robbed Machines:", [machine["Machine"] for machine in robbedMachines])
    print("Average robbery time:", round(avgRobberyTime, 2), "hours")

    robberyData = [[machine["Machine"], machine["Time"], machine["Distance"], machine["Path"]] for machine in robbedMachines]
    saveToCSV("machine_robbery_results.csv", robberyData, ["Machine", "Time (hrs)", "Distance (km)", "Path"], savePath)

def manageNetwork(network, startMachine, savePath=SAVE_PATH):
    # جداسازی نودهای اولویت‌دار و معمولی
    priorityMachines = [machine for machine in network.machines if machine.is_priority]
    normalMachines = [machine for machine in network.machines if not machine.is_priority and not machine.is_start]
    unvisitedMachines = priorityMachines + normalMachines

    visitedMachines = []
    currentMachine = startMachine
    totalDistance = 0
    totalTime = 0

    # مرحله 1: پردازش نودهای اولویت‌دار
    while priorityMachines:
        distances, previousMachines = calculateShortestPath(network, currentMachine)
        nextMachine = min(priorityMachines, key=lambda machine: distances[machine])

        travelTime = distances[nextMachine] * TIME_PER_KM
        totalTime += travelTime + SERVICE_TIME
        totalDistance += distances[nextMachine]

        path = findPath(previousMachines, nextMachine)
        visitedMachines.append({
            "Machine": nextMachine.id,
            "Path": [machine.id for machine in path],
            "Time": round(totalTime, 2),
            "Distance": distances[nextMachine],
        })

        priorityMachines.remove(nextMachine)
        currentMachine = nextMachine

    # مرحله 2: بررسی اینکه آیا نود شروع پردازش شده است یا خیر
    if startMachine not in [machine["Machine"] for machine in visitedMachines]:
        normalMachines.append(startMachine)

    # مرحله 3: پردازش نودهای معمولی
    while normalMachines:
        distances, previousMachines = calculateShortestPath(network, currentMachine)
        nextMachine = min(normalMachines, key=lambda machine: distances[machine])

        travelTime = distances[nextMachine] * TIME_PER_KM
        totalTime += travelTime + SERVICE_TIME
        totalDistance += distances[nextMachine]

        path = findPath(previousMachines, nextMachine)
        visitedMachines.append({
            "Machine": nextMachine.id,
            "Path": [machine.id for machine in path],
            "Time": round(totalTime, 2),
            "Distance": distances[nextMachine],
        })

        normalMachines.remove(nextMachine)
        currentMachine = nextMachine

    # محاسبه زمان میانگین
    avgTime = totalTime / len(visitedMachines)

    print("\n----- Managing Network -----")
    print("\nVisited Machines in order:")
    for machine in visitedMachines:
        print(f"Machine {machine['Machine']}, reached via path {machine['Path']} at time {machine['Time']} and distance: {machine['Distance']}")

    print("\nList of Machines:", [machine["Machine"] for machine in visitedMachines])
    print("Average time:", round(avgTime, 2), "hours")

    refillData = [[machine["Machine"], machine["Time"], machine["Distance"], machine["Path"]] for machine in visitedMachines]
    saveToCSV("machine_service_results.csv", refillData, ["Machine", "Time (hrs)", "Distance (km)", "Path"], savePath)

    # مرحله 4: شبیه‌سازی سرقت پس از سرویس‌دهی
    robberySimulation(network, startMachine, totalTime, savePath)
    
# -------------------------- Main Program -------------------------------
network = createNetwork(numMachines=NUM_ATM)

priorityMachines = random.sample(network.machines, random.randint(2, 4))
for machine in priorityMachines:
    machine.is_priority = True

startMachine = random.choice(network.machines)
startMachine.is_start = True

# Start the simulation and process the machines
manageNetwork(network, startMachine, SAVE_PATH)
displayNetwork(network)