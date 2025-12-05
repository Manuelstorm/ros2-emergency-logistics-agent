(define (problem instance3)
    (:domain logistic)
    (:requirements :negative-preconditions :universal-preconditions :disjunctive-preconditions :equality)
    (:objects
        box1 box2 box3 box4 - box
        depot pos1 pos2 pos3 pos4 pos5 pos6 pos7 - location
        person1 person2 person3 person4 person5 person6 person7 person8 - person
        banana1 banana2 banana3 banana4 banana5 banana6 banana7 banana8 - food
        aspirine1 aspirine2 aspirine3 aspirine4 aspirine5 aspirine6 aspirine7 aspirine8 - medicine
        scissor1 scissor2 scissor3 scissor4 scissor5 scissor6 scissor7 scissor8 - tools
        robot1 robot2 - robot
        carrier1 carrier2 - carrier
        s11 s12 s21 s22 - slot
    )

    (:init
        (at box1 depot)
        (at box2 depot)
        (at box3 depot)
        (at box4 depot)
        (empty box1)(empty box2)(empty box3)(empty box4)

        (at robot1 pos3)
        (at robot2 pos2)
        (at carrier1 pos3)
        (at carrier2 pos2)

        (free s11)(free s12)(free s21)(free s22)
        (belongs carrier1 s11)(belongs carrier1 s12)(belongs carrier1 s21)(belongs carrier1 s22)

        (at banana1 depot)
        (at banana2 depot)
        (at banana3 depot)
        (at banana4 depot)
        (at banana5 depot)
        (at banana6 depot)
        (at banana7 depot)
        (at banana8 depot)
        (at aspirine1 depot)
        (at aspirine2 depot)
        (at aspirine3 depot)
        (at aspirine4 depot)
        (at aspirine5 depot)
        (at aspirine6 depot)
        (at aspirine7 depot)
        (at aspirine8 depot)
        (at scissor1 depot)
        (at scissor2 depot)
        (at scissor3 depot)
        (at scissor4 depot)
        (at scissor5 depot)
        (at scissor6 depot)
        (at scissor7 depot)
        (at scissor8 depot)

        (assignable aspirine1)
        (assignable aspirine2)
        (assignable aspirine3)
        (assignable aspirine4)
        (assignable aspirine5)
        (assignable aspirine6)
        (assignable aspirine7)
        (assignable aspirine8)
        (assignable banana1)
        (assignable banana2)
        (assignable banana3)
        (assignable banana4)
        (assignable banana5)
        (assignable banana6)
        (assignable banana7)
        (assignable banana8)
        (assignable scissor1)
        (assignable scissor2)
        (assignable scissor3)
        (assignable scissor4)
        (assignable scissor5)
        (assignable scissor6)
        (assignable scissor7)
        (assignable scissor8)

        (at person1 pos1)
        (at person2 pos1)
        (at person3 pos2)
        (at person4 pos3)
        (at person5 pos4)
        (at person6 pos5)
        (at person7 pos6)
        (at person8 pos7)

        (needs-food person1)
        (needs-tools person1)

        (needs-medicine person2)

        (needs-medicine person3)

        (needs-medicine person4)
        (needs-food person4)

        (needs-medicine person5)
        (needs-food person5)
        (needs-tools person5)

        (needs-medicine person6)
        (needs-food person6)
        (needs-tools person6)

        (needs-medicine person7)
        (needs-food person7)
        (needs-tools person7)

        (needs-medicine person8)
        (needs-food person8)
        (needs-tools person8)

    )

    (:goal

        (forall
            (?p - person)
            (and
                (not (needs-food ?p))
                (not (needs-medicine ?p))
                (not (needs-tools ?p))
            )
        )

    )

)