(define (problem Problem1-demo)
    (:domain TelepresenceDomainv14)
    (:objects
      l1 - Location
      l2 - Location
      l3 - Location
      user1 - User
      user2 - User
      cs1 - ChargingStation
      l0 - Location
      r1 - Robot
      s1 - TelepresenceSession
      s2 - TelepresenceSession
      s3 - TelepresenceSession
      user3 - User
    )
    (:init
      (at user1 l1)
      (at user2 l2)
      (at user3 l3)
      (connected l3 l2)
      (connected l2 l3)
      (at r1 l0)
      (availableat cs1 l0)
      (connected l1 l2)
      (connected l2 l1)
      (connected l2 l0)
      (connected l0 l2)
      (connected l1 l0)
      (connected l0 l1)
      (available user1)
      (notinteracting user1)
      (available user2)
      (notinteracting user2)
      (idle cs1)
      (= (batterylevel r1) 50)
      (= (batterycapacity r1) 50)
      (= (velocity r1) 10.00)
      (= (consumptionratio r1) 10.00)
      (ready r1)
      (= (rechargeratio r1) 1.00)
      (= (consumptionratiosession r1) 10.00)
      (= (distance l0 l1) 20)
      (= (distance l1 l0) 20)
      (= (distance l1 l2) 20)
      (= (distance l2 l1) 20)
      (= (distance l2 l3) 20)
      (= (distance l3 l2) 20)
      (= (distance l0 l2) 20)
      (= (distance l2 l0) 20)
      (= (distancetostation l0 cs1) 0)
      (= (distancetostation l1 cs1) 20)
      (= (distancetostation l2 cs1) 60)
      (= (distancetostation l3 cs1) 40)
      (localuser s1 user1)
      (sessionlocation s1 l1)
      (= (sessionduration s1) 20)
      (localuser s2 user2)
      (sessionlocation s2 l2)
      (= (sessionduration s2) 20)
      (localuser s3 user3)
      (sessionlocation s3 l3)
      (= (sessionduration s3) 20)
      (available user3)
      (notinteracting user3)
      (at 100.00 (mustbedone s1))
      (at 130.00 (not (mustbedone s1)))
      (at 30.00 (mustbedone s2))
      (at 60.00 (not (mustbedone s2)))
      (at 200.00 (mustbedone s3))
      (at 230.00 (not (mustbedone s3)))
    )
    (:goal
      (and
        (at r1 l0)
        (done s1)
        (done s2)
      )
    )
)
