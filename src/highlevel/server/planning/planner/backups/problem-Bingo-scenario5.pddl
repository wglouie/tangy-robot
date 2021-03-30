(define (problem Problem5_1)
    (:domain MultiActivityDomainv6)
    (:objects
      l1 - Location
      l2 - Location
      l3 - Location
      l4 - Location
      l5 - Location
      l6 - Location
      l7 - Location
      l8 - Location
      l9 - Location
      user1 - User
      user2 - User
      user3 - User
      user4 - User
      cs1 - ChargingStation
      l0 - Location
      r1 - Robot
      l12 - Location
      l13 - Location
      l14 - Location
      l15 - Location
      l16 - Location
      l17 - Location
      l18 - Location
      l19 - Location
      user5 - User
      user6 - User
      user7 - User
      user8 - User
      g1 - BingoGame
      g2 - BingoGame
      lgame - Location
      user9 - User
      user10 - User
      user11 - User
      user12 - User
      g3 - BingoGame
      g4 - BingoGame
      user13 - User
      user14 - User
      user15 - User
      user16 - User
      l22 - Location
      l24 - Location
      l26 - Location
      l28 - Location
      g5 - BingoGame
      user17 - User
      user18 - User
      user19 - User
      user20 - User
    )
    (:init
      (connected l3 l1)
      (connected l1 l3)
      (connected l2 l3)
      (connected l3 l2)
      (connected l5 l3)
      (connected l3 l5)
      (connected l4 l5)
      (connected l5 l4)
      (connected l7 l5)
      (connected l5 l7)
      (connected l6 l7)
      (connected l7 l6)
      (connected l9 l7)
      (connected l7 l9)
      (connected l8 l9)
      (connected l9 l8)
      (at user1 l2)
      (at user2 l4)
      (at user3 l6)
      (at user4 l8)
      (connected l1 l0)
      (connected l0 l1)
      (at r1 l0)
      (availableat cs1 l0)
      (connected l12 l13)
      (connected l13 l12)
      (connected l15 l13)
      (connected l13 l15)
      (connected l14 l15)
      (connected l15 l14)
      (connected l17 l15)
      (connected l15 l17)
      (connected l16 l17)
      (connected l17 l16)
      (connected l19 l17)
      (connected l17 l19)
      (connected l18 l19)
      (connected l19 l18)
      (connected l13 l1)
      (connected l1 l13)
      (at user5 l12)
      (at user6 l14)
      (at user7 l16)
      (at user8 l18)
      (participant g1 user4)
      (participant g1 user3)
      (participant g1 user2)
      (participant g1 user1)
      (participant g2 user5)
      (participant g2 user6)
      (participant g2 user7)
      (participant g2 user8)
      (connected l1 lgame)
      (connected lgame l1)
      (participant g3 user9)
      (participant g3 user10)
      (participant g3 user12)
      (participant g3 user11)
      (participant g4 user13)
      (participant g4 user14)
      (participant g4 user15)
      (participant g4 user16)
      (participant g5 user17)
      (participant g5 user18)
      (participant g5 user19)
      (participant g5 user20)
      (connected l22 l13)
      (connected l13 l22)
      (connected l24 l15)
      (connected l15 l24)
      (connected l26 l17)
      (connected l17 l26)
      (connected l28 l19)
      (connected l19 l28)
      (at user17 l22)
      (at user18 l24)
      (at user19 l26)
      (at user20 l28)
      (at user12 l9)
      (at user11 l7)
      (at user10 l5)
      (at user9 l3)
      (at user13 l13)
      (at user14 l15)
      (at user15 l17)
      (at user16 l19)
      (notinteracting user1)
      (notinteracting user2)
      (notinteracting user3)
      (notinteracting user4)
      (idle cs1)
      (= (batterylevel r1) 1000)
      (= (batterycapacity r1) 1000)
      (= (velocity r1) 10.00)
      (= (consumptionratio r1) 10.00)
      (ready r1)
      (= (rechargeratio r1) 1.00)
      (= (consumptionratiosession r1) 10.00)
      (= (distance l0 l1) 20)
      (= (distance l1 l0) 20)
      (= (distance l1 l3) 20)
      (= (distance l3 l1) 20)
      (= (distance l2 l3) 20)
      (= (distance l3 l2) 20)
      (= (distance l3 l5) 20)
      (= (distance l5 l3) 20)
      (= (distance l4 l5) 20)
      (= (distance l5 l4) 20)
      (= (distance l5 l7) 20)
      (= (distance l7 l5) 20)
      (= (distance l6 l7) 20)
      (= (distance l7 l6) 20)
      (= (distance l7 l9) 20)
      (= (distance l9 l7) 20)
      (= (distance l8 l9) 20)
      (= (distance l9 l8) 20)
      (= (distance l1 l13) 20)
      (= (distance l13 l1) 20)
      (= (distance l12 l13) 20)
      (= (distance l13 l12) 20)
      (= (distance l13 l15) 20)
      (= (distance l15 l13) 20)
      (= (distance l14 l15) 20)
      (= (distance l15 l14) 20)
      (= (distance l15 l17) 20)
      (= (distance l17 l15) 20)
      (= (distance l16 l17) 20)
      (= (distance l17 l16) 20)
      (= (distance l17 l19) 20)
      (= (distance l19 l17) 20)
      (= (distance l18 l19) 20)
      (= (distance l19 l18) 20)
      (= (distance l1 lgame) 20)
      (= (distance lgame l1) 20)
      (= (distance l22 l13) 20)
      (= (distance l13 l22) 20)
      (= (distance l24 l15) 20)
      (= (distance l15 l24) 20)
      (= (distance l26 l17) 20)
      (= (distance l17 l26) 20)
      (= (distance l28 l19) 20)
      (= (distance l19 l28) 20)
      (= (distancetostation l0 cs1) 0)
      (= (distancetostation l1 cs1) 20)
      (= (distancetostation l2 cs1) 60)
      (= (distancetostation l3 cs1) 40)
      (= (distancetostation l4 cs1) 80)
      (= (distancetostation l5 cs1) 60)
      (= (distancetostation l6 cs1) 100)
      (= (distancetostation l7 cs1) 80)
      (= (distancetostation l8 cs1) 120)
      (= (distancetostation l9 cs1) 100)
      (= (distancetostation l12 cs1) 60)
      (= (distancetostation l13 cs1) 40)
      (= (distancetostation l14 cs1) 80)
      (= (distancetostation l15 cs1) 60)
      (= (distancetostation l16 cs1) 100)
      (= (distancetostation l17 cs1) 80)
      (= (distancetostation l18 cs1) 120)
      (= (distancetostation l19 cs1) 100)
      (= (distancetostation lgame cs1) 40)
      (= (distancetostation l22 cs1) 60)
      (= (distancetostation l24 cs1) 80)
      (= (distancetostation l26 cs1) 100)
      (= (distancetostation l28 cs1) 120)
      (notinteracting user5)
      (notinteracting user6)
      (notinteracting user7)
      (notinteracting user8)
      (gamelocation g1 lgame)
      (mustbedone_game g1)
      (= (gameduration g1) 60)
      (not_done_game g1)
      (need_reminder g1 user1)
      (need_reminder g1 user2)
      (need_reminder g1 user3)
      (need_reminder g1 user4)
      (= (reminder_duration g1) 2)
      (gamelocation g2 lgame)
      (mustbedone_game g2)
      (= (gameduration g2) 60)
      (not_done_game g2)
      (need_reminder g2 user5)
      (need_reminder g2 user6)
      (need_reminder g2 user7)
      (need_reminder g2 user8)
      (= (reminder_duration g2) 2)
      (notinteracting user9)
      (notinteracting user10)
      (notinteracting user11)
      (notinteracting user12)
      (gamelocation g3 lgame)
      (mustbedone_game g3)
      (= (gameduration g3) 60)
      (not_done_game g3)
      (need_reminder g3 user9)
      (need_reminder g3 user10)
      (need_reminder g3 user11)
      (need_reminder g3 user12)
      (= (reminder_duration g3) 2)
      (gamelocation g4 lgame)
      (mustbedone_game g4)
      (= (gameduration g4) 60)
      (not_done_game g4)
      (need_reminder g4 user13)
      (need_reminder g4 user14)
      (need_reminder g4 user15)
      (need_reminder g4 user16)
      (= (reminder_duration g4) 2)
      (notinteracting user13)
      (notinteracting user14)
      (notinteracting user15)
      (notinteracting user16)
      (gamelocation g5 lgame)
      (mustbedone_game g5)
      (= (gameduration g5) 60)
      (not_done_game g5)
      (need_reminder g5 user17)
      (need_reminder g5 user18)
      (need_reminder g5 user19)
      (need_reminder g5 user20)
      (= (reminder_duration g5) 2)
      (notinteracting user17)
      (notinteracting user18)
      (notinteracting user19)
      (notinteracting user20)
      (at 60.00 (available user1))
      (at 60.00 (available user2))
      (at 60.00 (available user3))
      (at 60.00 (available user4))
      (at 60.00 (available user5))
      (at 60.00 (available user6))
      (at 60.00 (available user7))
      (at 60.00 (available user8))
      (at 60.00 (available user9))
      (at 60.00 (available user10))
      (at 60.00 (available user11))
      (at 60.00 (available user12))
      (at 60.00 (available user13))
      (at 60.00 (available user14))
      (at 60.00 (available user15))
      (at 60.00 (available user16))
      (at 60.00 (available user20))
      (at 60.00 (available user19))
      (at 60.00 (available user18))
      (at 60.00 (available user17))
      (at 241.00 (not (available user8)))
      (at 241.00 (not (available user7)))
      (at 241.00 (not (available user6)))
      (at 241.00 (not (available user5)))
      (at 241.00 (not (available user1)))
      (at 241.00 (not (available user2)))
      (at 241.00 (not (available user3)))
      (at 241.00 (not (available user4)))
      (at 241.00 (not (available user12)))
      (at 241.00 (not (available user11)))
      (at 241.00 (not (available user10)))
      (at 241.00 (not (available user9)))
      (at 241.00 (not (available user13)))
      (at 241.00 (not (available user14)))
      (at 241.00 (not (available user15)))
      (at 241.00 (not (available user16)))
      (at 241.00 (not (available user20)))
      (at 241.00 (not (available user19)))
      (at 241.00 (not (available user18)))
      (at 241.00 (not (available user17)))
      (at 300.00 (available user8))
      (at 300.00 (available user7))
      (at 300.00 (available user6))
      (at 300.00 (available user5))
      (at 300.00 (available user1))
      (at 300.00 (available user2))
      (at 300.00 (available user3))
      (at 300.00 (available user4))
      (at 300.00 (available user12))
      (at 300.00 (available user11))
      (at 300.00 (available user10))
      (at 300.00 (available user9))
      (at 300.00 (available user13))
      (at 300.00 (available user14))
      (at 300.00 (available user15))
      (at 300.00 (available user16))
      (at 300.00 (available user20))
      (at 300.00 (available user19))
      (at 300.00 (available user18))
      (at 300.00 (available user17))
      (at 541.00 (not (available user8)))
      (at 541.00 (not (available user7)))
      (at 541.00 (not (available user6)))
      (at 541.00 (not (available user5)))
      (at 541.00 (not (available user1)))
      (at 541.00 (not (available user2)))
      (at 541.00 (not (available user3)))
      (at 541.00 (not (available user4)))
      (at 541.00 (not (available user12)))
      (at 541.00 (not (available user11)))
      (at 541.00 (not (available user10)))
      (at 541.00 (not (available user9)))
      (at 541.00 (not (available user13)))
      (at 541.00 (not (available user14)))
      (at 541.00 (not (available user15)))
      (at 541.00 (not (available user16)))
      (at 541.00 (not (available user20)))
      (at 541.00 (not (available user19)))
      (at 541.00 (not (available user18)))
      (at 541.00 (not (available user17)))
      (at 600.00 (available user8))
      (at 600.00 (available user7))
      (at 600.00 (available user6))
      (at 600.00 (available user5))
      (at 600.00 (available user1))
      (at 600.00 (available user2))
      (at 600.00 (available user3))
      (at 600.00 (available user4))
      (at 600.00 (available user12))
      (at 600.00 (available user11))
      (at 600.00 (available user10))
      (at 600.00 (available user9))
      (at 600.00 (available user13))
      (at 600.00 (available user14))
      (at 600.00 (available user15))
      (at 600.00 (available user16))
      (at 600.00 (available user20))
      (at 600.00 (available user19))
      (at 600.00 (available user18))
      (at 600.00 (available user17))
      (at 665.00 (not (available user8)))
      (at 665.00 (not (available user7)))
      (at 665.00 (not (available user6)))
      (at 665.00 (not (available user5)))
      (at 665.00 (not (available user1)))
      (at 665.00 (not (available user2)))
      (at 665.00 (not (available user3)))
      (at 665.00 (not (available user4)))
      (at 665.00 (not (available user12)))
      (at 665.00 (not (available user11)))
      (at 665.00 (not (available user10)))
      (at 665.00 (not (available user9)))
      (at 665.00 (not (available user13)))
      (at 665.00 (not (available user14)))
      (at 665.00 (not (available user15)))
      (at 665.00 (not (available user16)))
      (at 665.00 (not (available user20)))
      (at 665.00 (not (available user19)))
      (at 665.00 (not (available user18)))
      (at 665.00 (not (available user17)))
      (at 61 (not (available user1)))
      (at 61 (not (available user3)))
      (at 61 (not (available user6)))
      (at 61 (not (available user10)))
      (at 61 (not (available user11)))
      (at 120 (available user1))
      (at 120 (available user3))
      (at 120 (available user6))
      (at 120 (available user10))
      (at 120 (available user11))
      (at 121.00 (not (available user2)))
      (at 121.00 (not (available user4)))
      (at 121.00 (not (available user6)))
      (at 121.00 (not (available user8)))
      (at 121.00 (not (available user9)))
      (at 121.00 (not (available user12)))
      (at 121.00 (not (available user14)))
      (at 121.00 (not (available user15)))
      (at 180.00 (available user8))
      (at 180.00 (available user6))
      (at 180.00 (available user2))
      (at 180.00 (available user4))
      (at 180.00 (available user9))
      (at 180.00 (available user12))
      (at 180.00 (available user14))
      (at 180.00 (available user15))
      (at 181.00 (not (available user5)))
      (at 181.00 (not (available user7)))
      (at 181.00 (not (available user13)))
      (at 181.00 (not (available user16)))
      (at 240.00 (available user7))
      (at 240.00 (available user5))
      (at 240.00 (available user13))
      (at 240.00 (available user16))
      (at 301.00 (not (available user1)))
      (at 301.00 (not (available user3)))
      (at 301.00 (not (available user14)))
      (at 301.00 (not (available user15)))
      (at 301.00 (not (available user20)))
      (at 301.00 (not (available user17)))
      (at 360.00 (available user3))
      (at 360.00 (available user1))
      (at 360.00 (available user14))
      (at 360.00 (available user15))
      (at 360.00 (available user20))
      (at 360.00 (available user17))
      (at 361.00 (not (available user2)))
      (at 361.00 (not (available user4)))
      (at 361.00 (not (available user13)))
      (at 361.00 (not (available user16)))
      (at 361.00 (not (available user18)))
      (at 420.00 (available user4))
      (at 420.00 (available user2))
      (at 420.00 (available user13))
      (at 420.00 (available user16))
      (at 421.00 (not (available user6)))
      (at 421.00 (not (available user8)))
      (at 421.00 (not (available user9)))
      (at 421.00 (not (available user10)))
      (at 421.00 (not (available user12)))
      (at 421.00 (not (available user14)))
      (at 480.00 (available user6))
      (at 480.00 (available user8))
      (at 480.00 (available user12))
      (at 480.00 (available user10))
      (at 480.00 (available user9))
      (at 480.00 (available user14))
      (at 481.00 (not (available user2)))
      (at 481.00 (not (available user5)))
      (at 481.00 (not (available user7)))
      (at 481.00 (not (available user19)))
      (at 540.00 (available user7))
      (at 540.00 (available user5))
      (at 540.00 (available user2))
      (at 540.00 (available user18))
      (at 540.00 (available user19))
      (at 601.00 (not (available user11)))
      (at 601.00 (not (available user10)))
      (at 601.00 (not (available user17)))
      (at 601.00 (not (available user19)))
      (at 601.00 (not (available user20)))
      (at 660.00 (available user11))
      (at 660.00 (available user10))
      (at 660.00 (available user17))
      (at 660.00 (available user19))
      (at 660.00 (available user20))
    )
    (:goal
      (and
        (at r1 l0)
        (done_game g1)
        (done_game g2)
        (done_game g3)
        (done_game g4)
        (done_game g5)
      )
    )
)
