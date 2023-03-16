import matplotlib.pyplot as plt

class TimeWatch:
    def __init__(self, NofLastTimes):
        self.N = NofLastTimes
        self.n = 1

        self.numOfTimes = {}
        self.meanTimes = {}
        self.maxTimes = {}
        self.lastNTimes = {}
        self.times = {}

    def createCounter(self, counterNames):
        for name in counterNames:
            self.numOfTimes[name] = 0
            self.meanTimes[name] = 0.0
            self.maxTimes[name] = 0.0
            self.lastNTimes[name] = self.N * [0.0]
            self.times[name] = [0.0]

    def update(self, counterName, time):
        K = self.numOfTimes[counterName]

        self.meanTimes[counterName] = (self.meanTimes[counterName]*K + time)/(K+1)
        self.maxTimes[counterName] = max(self.maxTimes[counterName], time)
        self.lastNTimes[counterName] = self.lastNTimes[counterName][1:] + [time]
        self.times[counterName] = self.times[counterName] + [time]

        self.numOfTimes[counterName] += 1

    def plotTimes(self, counterNames):
        plt.figure(self.n)

        for counter in counterNames:
            plt.plot(list(range(len(self.times[counter]))), self.times[counter], label=counter)

        plt.grid(True)
        plt.xlabel("k")
        plt.ylabel("time (s)")
        plt.legend()
        plt.savefig(
            "timetest_graph{}.pdf".format(self.n)
            ,
            bbox_inches="tight",
        )

        self.n += 1
