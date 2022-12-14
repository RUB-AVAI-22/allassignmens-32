import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import math


def plot(ranges):
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    ax.set_theta_zero_location('W', offset=90)
    ax.set_theta_direction(-1)
    ax.set_rorigin(-1)
    c = ax.scatter([i * math.pi/180 for i in range(0, 360)], ranges)

    # camera angle probably 62.2°
    ax.set_thetamin(220)
    ax.set_thetamax(140)
    ax.set_ylim([0, 1])

    # Set the title of the polar plot
    plt.title('LIDAR')

    # Plot a circle with radius 2 using polar form
    #rads = [i for i in range(0, 361)]

    #for radian in rads:
    #for i in range(0, 361):
    #    plt.polar(rads[i], ranges[i], 'o')

    # Display the Polar plot
    plt.show()

def r1():
    plt.subplot(111, polar=True)
    plt.show()

ranges = [[0.0, 2.8970000743865967, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5640000104904175, 0.0, 0.0, 3.7109999656677246, 3.6500000953674316, 3.575000047683716, 3.552999973297119, 3.500999927520752, 1.6790000200271606, 1.6100000143051147, 1.6119999885559082, 1.559000015258789, 1.6119999885559082, 1.562000036239624, 1.5729999542236328, 1.5440000295639038, 1.684000015258789, 3.1700000762939453, 3.1570000648498535, 3.1419999599456787, 3.119999885559082, 3.0880000591278076, 3.0880000591278076, 3.0439999103546143, 3.0320000648498535, 3.0209999084472656, 2.9730000495910645, 2.9690001010894775, 2.9619998931884766, 2.953000068664551, 2.9609999656677246, 2.9260001182556152, 2.9200000762939453, 2.9110000133514404, 2.9140000343322754, 2.8959999084472656, 2.9119999408721924, 2.9000000953674316, 2.9030001163482666, 2.9059998989105225, 2.884999990463257, 2.877000093460083, 2.9070000648498535, 2.927999973297119, 2.9040000438690186, 2.9179999828338623, 2.9049999713897705, 2.934000015258789, 2.930999994277954, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2430000305175781, 1.253000020980835, 1.2610000371932983, 1.281000018119812, 1.2239999771118164, 1.2330000400543213, 1.215000033378601, 1.187000036239624, 1.2389999628067017, 1.2120000123977661, 1.1430000066757202, 1.1200000047683716, 1.1649999618530273, 1.0750000476837158, 1.0789999961853027, 1.1169999837875366, 1.059000015258789, 1.0820000171661377, 1.0119999647140503, 1.0110000371932983, 1.0199999809265137, 0.9710000157356262, 0.9710000157356262, 1.899999976158142, 1.8650000095367432, 1.8389999866485596, 1.8240000009536743, 1.7979999780654907, 1.7790000438690186, 1.7649999856948853, 1.7280000448226929, 1.7139999866485596, 1.7139999866485596, 1.7089999914169312, 1.690999984741211, 1.6690000295639038, 1.6579999923706055, 1.652999997138977, 1.6440000534057617, 1.6200000047683716, 1.6230000257492065, 1.5980000495910645, 1.6130000352859497, 1.590999960899353, 1.5759999752044678, 1.559000015258789, 1.565999984741211, 1.5509999990463257, 1.559999942779541, 1.5399999618530273, 1.5230000019073486, 1.5390000343322754, 1.524999976158142, 1.5299999713897705, 1.5290000438690186, 1.5290000438690186, 1.5230000019073486, 1.5299999713897705, 1.5190000534057617, 1.5190000534057617, 1.5160000324249268, 1.5099999904632568, 1.5219999551773071, 1.5180000066757202, 1.5210000276565552, 1.5190000534057617, 0.5809999704360962, 0.5809999704360962, 0.578000009059906, 0.5789999961853027, 0.5820000171661377, 0.5879999995231628, 0.5950000286102295, 0.0, 1.5640000104904175, 1.5729999542236328, 1.5789999961853027, 1.5959999561309814, 1.6100000143051147, 1.6169999837875366, 1.6180000305175781, 1.6269999742507935, 1.6510000228881836, 1.656999945640564, 1.656000018119812, 1.6920000314712524, 1.6890000104904175, 1.6979999542236328, 1.7209999561309814, 1.7269999980926514, 1.746999979019165, 1.7580000162124634, 1.7699999809265137, 0.5080000162124634, 0.5080000162124634, 0.5040000081062317, 0.503000020980835, 0.503000020980835, 0.5059999823570251, 0.5099999904632568, 0.515999972820282, 0.0, 2.0810000896453857, 2.0380001068115234, 2.0969998836517334, 2.11299991607666, 2.071000099182129, 2.178999900817871, 2.24399995803833, 2.2809998989105225, 2.428999900817871, 2.375999927520752, 2.447000026702881, 2.4679999351501465, 2.561000108718872, 2.63700008392334, 2.691999912261963, 2.7909998893737793, 2.884999990463257, 2.9549999237060547, 3.0179998874664307, 3.132999897003174, 0.6100000143051147, 0.6100000143051147, 0.6069999933242798, 0.6079999804496765, 0.6100000143051147, 0.6119999885559082, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0280001163482666, 1.937000036239624, 1.9759999513626099, 2.003000020980835, 2.509999990463257, 0.0, 0.0, 3.2279999256134033, 0.0, 0.0, 1.934999942779541, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1440000534057617, 0.0, 1.9550000429153442, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7450000047683716, 2.8889999389648438, 2.924999952316284, 3.062999963760376, 0.0, 1.590000033378601, 0.0, 0.0, 0.0, 0.0, 0.0, 2.9709999561309814, 0.0, 2.3440001010894775, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7709999084472656, 0.0, 0.0, 0.0, 0.0, 3.257999897003174, 3.368000030517578, 0.0, 0.0, 0.0, 0.0, 0.0, 2.953000068664551, 0.0, 0.0, 0.0, 3.2829999923706055, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1069999933242798, 0.8980000019073486, 0.8980000019073486, 0.0, 0.0, 1.187000036239624, 1.1959999799728394, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7020000219345093, 1.6399999856948853, 0.0, 0.0, 0.9259999990463257, 0.9380000233650208, 0.6940000057220459, 0.0, 0.0, 2.3510000705718994, 2.3499999046325684, 2.4049999713897705, 2.562000036239624, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.424999952316284, 3.385999917984009, 0.0, 0.0, 1.9299999475479126, 1.878000020980835, 1.8839999437332153, 1.9079999923706055, 1.8329999446868896, 0.0, 0.0, 0.0, 2.86899995803833],
          [0.0, 2.875, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.6630001068115234, 3.6519999504089355, 3.559000015258789, 3.5829999446868896, 3.4809999465942383, 1.7239999771118164, 1.6230000257492065, 1.6080000400543213, 1.6019999980926514, 1.5859999656677246, 1.527999997138977, 1.6019999980926514, 1.5479999780654907, 1.6339999437332153, 3.1740000247955322, 3.1700000762939453, 3.1429998874664307, 3.11299991607666, 3.0910000801086426, 3.072000026702881, 3.056999921798706, 3.002000093460083, 2.992000102996826, 2.9660000801086426, 2.984999895095825, 2.9860000610351562, 2.937000036239624, 2.933000087738037, 2.933000087738037, 2.930000066757202, 2.9040000438690186, 2.9149999618530273, 2.8929998874664307, 2.9000000953674316, 2.9070000648498535, 2.9030001163482666, 2.9049999713897705, 2.9159998893737793, 2.885999917984009, 2.933000087738037, 2.9189999103546143, 2.9079999923706055, 2.9040000438690186, 2.9170000553131104, 2.930999994277954, 2.9079999923706055, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3270000219345093, 1.2699999809265137, 1.3730000257492065, 1.2899999618530273, 1.2309999465942383, 1.2350000143051147, 1.2610000371932983, 1.1890000104904175, 1.1519999504089355, 1.149999976158142, 1.1579999923706055, 1.1510000228881836, 1.1160000562667847, 1.065000057220459, 1.0950000286102295, 1.0720000267028809, 1.0670000314712524, 1.0290000438690186, 1.0700000524520874, 1.0230000019073486, 1.024999976158142, 1.0019999742507935, 1.0019999742507935, 1.8860000371932983, 1.8630000352859497, 1.8329999446868896, 1.8109999895095825, 1.8170000314712524, 1.7829999923706055, 1.7649999856948853, 1.7369999885559082, 1.718000054359436, 1.722000002861023, 1.7100000381469727, 1.6950000524520874, 1.6679999828338623, 1.6670000553131104, 1.6510000228881836, 1.6239999532699585, 1.6390000581741333, 1.61899995803833, 1.6069999933242798, 1.593000054359436, 1.5920000076293945, 1.5759999752044678, 1.559000015258789, 1.555999994277954, 1.5429999828338623, 1.5529999732971191, 1.5410000085830688, 1.5479999780654907, 1.5369999408721924, 1.534000039100647, 1.5329999923706055, 1.5230000019073486, 1.5299999713897705, 1.5180000066757202, 1.5110000371932983, 1.5140000581741333, 1.5230000019073486, 1.5260000228881836, 1.5119999647140503, 1.5130000114440918, 1.5190000534057617, 1.503999948501587, 1.5230000019073486, 0.5809999704360962, 0.5809999704360962, 0.578000009059906, 0.578000009059906, 0.5809999704360962, 0.5860000252723694, 0.593999981880188, 0.7039999961853027, 1.5540000200271606, 1.5609999895095825, 1.5679999589920044, 1.5980000495910645, 1.6139999628067017, 1.6089999675750732, 1.6139999628067017, 1.6469999551773071, 1.6410000324249268, 1.6440000534057617, 1.6640000343322754, 1.6859999895095825, 1.7039999961853027, 1.7020000219345093, 1.715000033378601, 1.718999981880188, 1.7410000562667847, 1.7669999599456787, 1.7610000371932983, 0.5090000033378601, 0.5090000033378601, 0.5049999952316284, 0.5040000081062317, 0.5040000081062317, 0.5070000290870667, 0.5109999775886536, 0.515999972820282, 0.0, 2.075000047683716, 2.055999994277954, 2.0889999866485596, 2.1040000915527344, 2.052999973297119, 2.194000005722046, 2.244999885559082, 2.2890000343322754, 2.441999912261963, 2.3929998874664307, 2.447000026702881, 2.493000030517578, 2.566999912261963, 2.6500000953674316, 2.687000036239624, 2.765000104904175, 2.8929998874664307, 2.9769999980926514, 3.0220000743865967, 3.13700008392334, 0.6100000143051147, 0.6100000143051147, 0.6079999804496765, 0.6079999804496765, 0.6100000143051147, 0.6129999756813049, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9459999799728394, 1.9470000267028809, 1.9190000295639038, 2.003999948501587, 2.618000030517578, 0.0, 0.0, 3.2660000324249268, 0.0, 0.0, 1.9479999542236328, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0929999351501465, 0.0, 1.9529999494552612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.9070000648498535, 2.9159998893737793, 3.0789999961853027, 0.0, 1.6019999980926514, 0.0, 0.0, 0.0, 2.565999984741211, 0.0, 2.9489998817443848, 2.944999933242798, 2.2070000171661377, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.674999952316284, 0.0, 0.0, 0.0, 0.0, 3.309999942779541, 3.3359999656677246, 0.0, 0.0, 0.0, 0.0, 0.0, 2.9590001106262207, 0.0, 0.0, 0.0, 3.4130001068115234, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.093999981880188, 0.8140000104904175, 1.0549999475479126, 0.0, 0.0, 1.2940000295639038, 1.246999979019165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6729999780654907, 1.6649999618530273, 0.0, 0.0, 0.9169999957084656, 0.9259999990463257, 1.0390000343322754, 0.0, 0.0, 2.3499999046325684, 2.3389999866485596, 2.3889999389648438, 2.555000066757202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.434999942779541, 3.38100004196167, 0.0, 0.0, 1.9600000381469727, 1.871000051498413, 1.8769999742507935, 1.8660000562667847, 1.8650000095367432, 0.0, 0.0, 0.0, 2.808000087738037],
          [0.0, 2.8970000743865967, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7260000705718994, 3.6449999809265137, 3.5999999046325684, 3.5940001010894775, 3.490999937057495, 2.6449999809265137, 1.6629999876022339, 1.562000036239624, 1.5750000476837158, 1.5859999656677246, 1.559999942779541, 1.5499999523162842, 1.562999963760376, 1.6180000305175781, 3.1610000133514404, 3.1449999809265137, 3.1559998989105225, 3.1070001125335693, 3.0980000495910645, 3.0739998817443848, 3.0490000247955322, 3.0299999713897705, 3.0339999198913574, 2.9739999771118164, 2.9630000591278076, 3.0199999809265137, 2.940999984741211, 2.938999891281128, 2.921999931335449, 2.9130001068115234, 2.9030001163482666, 2.9159998893737793, 2.9040000438690186, 2.9089999198913574, 2.8989999294281006, 2.8919999599456787, 2.9059998989105225, 2.9000000953674316, 2.9079999923706055, 2.9049999713897705, 2.9070000648498535, 2.9200000762939453, 2.924999952316284, 2.930999994277954, 2.9240000247955322, 2.9149999618530273, 2.63700008392334, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2480000257492065, 1.2699999809265137, 1.2230000495910645, 1.2599999904632568, 1.2630000114440918, 1.1859999895095825, 1.2510000467300415, 1.2330000400543213, 1.1770000457763672, 1.180999994277954, 1.1440000534057617, 1.0789999961853027, 1.1100000143051147, 1.1160000562667847, 1.093000054359436, 1.090000033378601, 1.0390000343322754, 1.0850000381469727, 1.0570000410079956, 1.059999942779541, 1.0240000486373901, 0.9769999980926514, 0.9769999980926514, 1.9019999504089355, 1.875, 1.8359999656677246, 1.805999994277954, 1.8049999475479126, 1.7829999923706055, 1.7660000324249268, 1.7510000467300415, 1.722000002861023, 1.715999960899353, 1.7050000429153442, 1.687000036239624, 1.6720000505447388, 1.659999966621399, 1.6670000553131104, 1.6480000019073486, 1.6399999856948853, 1.6069999933242798, 1.6050000190734863, 1.5989999771118164, 1.597000002861023, 1.5770000219345093, 1.565000057220459, 1.565999984741211, 1.5399999618530273, 1.5490000247955322, 1.5379999876022339, 1.5460000038146973, 1.5240000486373901, 1.5329999923706055, 1.5260000228881836, 1.5260000228881836, 1.5190000534057617, 1.50600004196167, 1.5110000371932983, 1.5110000371932983, 1.5160000324249268, 1.5169999599456787, 1.4980000257492065, 1.5219999551773071, 1.5219999551773071, 1.503999948501587, 1.5130000114440918, 0.5809999704360962, 0.5809999704360962, 0.5789999961853027, 0.5789999961853027, 0.5809999704360962, 0.5860000252723694, 0.5910000205039978, 0.6539999842643738, 1.5670000314712524, 1.5759999752044678, 1.5579999685287476, 1.5829999446868896, 1.5859999656677246, 1.6019999980926514, 1.6050000190734863, 1.6369999647140503, 1.6360000371932983, 1.6469999551773071, 1.6720000505447388, 1.6779999732971191, 1.7029999494552612, 1.6950000524520874, 1.7089999914169312, 1.7330000400543213, 1.75, 1.7640000581741333, 1.7769999504089355, 0.5090000033378601, 0.5090000033378601, 0.5049999952316284, 0.5040000081062317, 0.5040000081062317, 0.5070000290870667, 0.5109999775886536, 0.5149999856948853, 0.6079999804496765, 2.0309998989105225, 2.0460000038146973, 2.0799999237060547, 2.118000030517578, 2.0969998836517334, 2.187999963760376, 2.2279999256134033, 2.2790000438690186, 2.305000066757202, 2.384000062942505, 2.447999954223633, 2.500999927520752, 2.562000036239624, 2.61899995803833, 2.674999952316284, 2.7809998989105225, 2.869999885559082, 2.9619998931884766, 3.0339999198913574, 3.125, 0.6100000143051147, 0.6100000143051147, 0.6079999804496765, 0.6079999804496765, 0.609000027179718, 0.6119999885559082, 0.6840000152587891, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9459999799728394, 1.9620000123977661, 1.9229999780654907, 1.9869999885559082, 2.5940001010894775, 0.0, 0.0, 3.4539999961853027, 0.0, 0.0, 1.9420000314712524, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1449999809265137, 0.0, 1.9450000524520874, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7999999523162842, 2.9200000762939453, 2.9230000972747803, 3.049999952316284, 0.0, 1.6349999904632568, 0.0, 0.0, 0.0, 2.509999990463257, 0.0, 2.992000102996826, 2.9800000190734863, 2.180000066757202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.752000093460083, 0.0, 0.0, 0.0, 0.0, 3.4189999103546143, 3.2339999675750732, 0.0, 0.0, 0.0, 0.0, 0.0, 2.9519999027252197, 0.0, 0.0, 0.0, 3.3299999237060547, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0010000467300415, 0.8920000195503235, 0.984000027179718, 0.0, 0.0, 1.3509999513626099, 1.2890000343322754, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6979999542236328, 1.6749999523162842, 0.0, 0.0, 0.8539999723434448, 0.843999981880188, 0.8339999914169312, 0.0, 0.0, 2.3350000381469727, 2.3329999446868896, 2.388000011444092, 2.5490000247955322, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.4489998817443848, 3.4179999828338623, 0.0, 0.0, 1.944000005722046, 1.875, 1.8730000257492065, 1.878999948501587, 1.843000054359436, 0.0, 0.0, 0.0, 2.88700008392334],
          [0.0, 2.865999937057495, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7149999141693115, 3.6710000038146973, 3.5769999027252197, 3.690000057220459, 3.4839999675750732, 3.3469998836517334, 1.6030000448226929, 1.597000002861023, 1.625, 1.63100004196167, 1.5240000486373901, 1.5069999694824219, 1.559999942779541, 1.590000033378601, 3.1989998817443848, 3.1589999198913574, 3.1570000648498535, 3.109999895095825, 3.1089999675750732, 3.0869998931884766, 3.062999963760376, 3.0299999713897705, 3.0, 2.9690001010894775, 2.9769999980926514, 2.9679999351501465, 2.9560000896453857, 2.9570000171661377, 2.931999921798706, 2.9260001182556152, 2.931999921798706, 2.9179999828338623, 2.9130001068115234, 2.890000104904175, 2.9070000648498535, 2.8919999599456787, 2.921999931335449, 2.890000104904175, 2.9030001163482666, 2.9040000438690186, 2.9140000343322754, 2.9130001068115234, 2.921999931335449, 2.8970000743865967, 2.931999921798706, 2.9110000133514404, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3899999856948853, 1.3830000162124634, 1.406999945640564, 1.274999976158142, 1.152999997138977, 1.187999963760376, 1.343999981880188, 1.2400000095367432, 1.2740000486373901, 1.2139999866485596, 1.1399999856948853, 1.121999979019165, 1.156000018119812, 1.1679999828338623, 1.090999960899353, 1.090000033378601, 1.093000054359436, 1.0540000200271606, 1.0490000247955322, 1.034999966621399, 1.0549999475479126, 1.0609999895095825, 0.972000002861023, 0.972000002861023, 1.8960000276565552, 1.8580000400543213, 1.8450000286102295, 1.8220000267028809, 1.8070000410079956, 1.781000018119812, 1.7690000534057617, 1.7330000400543213, 1.7209999561309814, 1.7100000381469727, 1.7079999446868896, 1.6859999895095825, 1.6660000085830688, 1.6610000133514404, 1.659999966621399, 1.6369999647140503, 1.6319999694824219, 1.621000051498413, 1.600000023841858, 1.593000054359436, 1.5880000591278076, 1.5800000429153442, 1.5670000314712524, 1.5549999475479126, 1.5429999828338623, 1.5499999523162842, 1.5390000343322754, 1.5410000085830688, 1.5299999713897705, 1.534999966621399, 1.5269999504089355, 1.5269999504089355, 1.5260000228881836, 1.5169999599456787, 1.5210000276565552, 1.5199999809265137, 1.5119999647140503, 1.5160000324249268, 1.5119999647140503, 1.5180000066757202, 1.5169999599456787, 1.5230000019073486, 1.5160000324249268, 0.5809999704360962, 0.5809999704360962, 0.5789999961853027, 0.5789999961853027, 0.5820000171661377, 0.5870000123977661, 0.593999981880188, 0.6850000023841858, 1.559000015258789, 1.565000057220459, 1.569000005722046, 1.5740000009536743, 1.5889999866485596, 1.6009999513626099, 1.6050000190734863, 1.6299999952316284, 1.6369999647140503, 1.6490000486373901, 1.6579999923706055, 1.6859999895095825, 1.6959999799728394, 1.694000005722046, 1.7120000123977661, 1.725000023841858, 1.753000020980835, 1.7610000371932983, 1.7660000324249268, 0.5080000162124634, 0.5080000162124634, 0.5040000081062317, 0.503000020980835, 0.5040000081062317, 0.5070000290870667, 0.5099999904632568, 0.5149999856948853, 0.0, 2.072000026702881, 2.053999900817871, 2.0759999752044678, 2.0980000495910645, 2.0810000896453857, 2.183000087738037, 2.240000009536743, 2.2690000534057617, 2.306999921798706, 2.38700008392334, 2.4539999961853027, 2.490999937057495, 2.5480000972747803, 2.634000062942505, 2.687000036239624, 2.7339999675750732, 2.869999885559082, 2.9489998817443848, 3.002000093460083, 3.1070001125335693, 0.6100000143051147, 0.6100000143051147, 0.6079999804496765, 0.6079999804496765, 0.609000027179718, 0.6110000014305115, 0.703000009059906, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9609999656677246, 1.937999963760376, 1.9259999990463257, 1.9919999837875366, 2.6429998874664307, 0.0, 0.0, 3.562999963760376, 0.0, 0.0, 1.940999984741211, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.075000047683716, 0.0, 1.9639999866485596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7630000114440918, 2.927999973297119, 2.946000099182129, 3.068000078201294, 0.0, 1.590000033378601, 0.0, 3.0439999103546143, 3.069000005722046, 2.5490000247955322, 0.0, 2.9649999141693115, 2.9639999866485596, 2.186000108718872, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.674999952316284, 0.0, 0.0, 0.0, 0.0, 3.315999984741211, 3.312000036239624, 0.0, 0.0, 0.0, 0.0, 2.882999897003174, 2.943000078201294, 0.0, 0.0, 0.0, 3.312999963760376, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9700000286102295, 0.9700000286102295, 0.8970000147819519, 0.0, 0.0, 1.2660000324249268, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6970000267028809, 1.6519999504089355, 0.0, 1.2869999408721924, 0.968999981880188, 0.7789999842643738, 0.9800000190734863, 0.0, 0.0, 2.3299999237060547, 2.3310000896453857, 2.4159998893737793, 2.569000005722046, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.4089999198913574, 3.4159998893737793, 0.0, 0.0, 1.9869999885559082, 1.8539999723434448, 1.8359999656677246, 1.8769999742507935, 1.8839999437332153, 0.0, 0.0, 0.0, 2.872999906539917]
          ]
plot(ranges[0])
