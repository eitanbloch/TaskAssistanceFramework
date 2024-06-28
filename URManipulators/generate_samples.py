import pickle
from time import sleep, time
from CameraController import CameraWrapper
from Robots import TaskRobot, AssistanceRobot

assistance_robot = AssistanceRobot()
task_robot = TaskRobot()


def generate_samples():
    samples = []
    print('starting counter')
    # sleep(10)
    while True:
        input()
        samples.append(assistance_robot.get_config())
        print(f'Length: {len(samples)}')
        print(samples)

generate_samples()
# [[-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597, -0.4274819532977503], [0.494315505027771, -0.8066403430751343, -0.004446625709533691, -1.210972325210907, -1.896071736012594, 0.5126177072525024]]
# [[1.1083424091339111, -0.8869695228389283, -0.0053337812423706055, -1.175297812824585, -2.3454323450671595, 0.9656500816345215], [0.804753839969635, -0.11332018793139653, 0.6386845747577112, -3.5985146961607875, -2.345687452946798, 0.05820784717798233], [0.7405092120170593, -1.0121575158885499, 0.6384099165545862, -2.695364614526266, -2.3456791082965296, 0.058171361684799194], [-0.41845876375307256, -0.9729586404613038, -0.004474210552871227, -0.9480648797801514, -1.4475892225848597, -0.4274819532977503], [0.494315505027771, -0.8066403430751343, -0.004446625709533691, -1.210972325210907, -1.896071736012594, 0.5126177072525024]]

# [[1.1115598678588867, -1.172658161526062, -0.01645125262439251, -1.3162224751761933, -1.8276990095721644, 0.5830123424530029], [1.1113834381103516, -1.172726796274521, -0.016502821817994118, -1.407630906706192, -1.782706085835592, 0.5829644203186035], [1.1116678714752197, -1.1369864803603669, -0.01613111048936844, -1.42645517111335, -2.141855541859762, 0.5830004215240479], [1.1177151203155518, -0.9190715116313477, -0.01606273651123047, -1.5822354755797328, -2.1243656317340296, 0.5834860801696777], [1.1173758506774902, -0.6588157576373597, -0.015970280393958092, -1.7741910419859828, -2.3131774107562464, 0.6182942986488342], [1.0722233057022095, -0.4512731593898316, -0.015922224149107933, -1.7810565433897914, -2.366791311894552, 1.0761826038360596], [0.9321948885917664, -0.3541247409633179, -0.01589825190603733, -1.6245233021178187, -2.366091791783468, 1.0787479877471924], [0.8644253611564636, -0.22924871862445073, -0.015831459313631058, -2.0665608845152796, -2.3677175680743616, 0.5781026482582092], [0.7871353030204773, -0.021220044498779345, -0.01582447811961174, -2.5048495731749476, -2.367075268422262, 0.4374757707118988], [0.7195990681648254, 0.04493491231884761, -0.015819372609257698, -2.6515332661070765, -2.175149742756979, 0.43690216541290283], [0.7675868272781372, 0.12758342802014155, -0.015800489112734795, -2.550619741479391, -2.1733220259295862, 0.43701013922691345], [0.758949339389801, 0.1467626529880981, -0.015855420380830765, -2.913814207116598, -2.416149441395895, 0.4364640712738037], [0.7628309726715088, 0.14657764017071528, -0.015922199934720993, -2.8650595150389613, -2.2634623686419886, 0.4350182116031647], [0.7625393271446228, 0.10826150953259273, -0.015674052760004997, -2.8395120106139125, -2.399479929600851, 0.414956659078598], [0.7855076193809509, 0.10286442815747066, -0.016102086752653122, -2.835318227807516, -2.3956356684314173, 0.38744857907295227], [0.7615655660629272, 0.11305288850750728, -0.015600309707224369, -2.787488122979635, -2.2013638655291956, 0.20387756824493408], [0.7359480261802673, 0.1324869829365234, -0.016018306836485863, -2.876286645928854, -2.2988370100604456, 0.27461209893226624], [0.7544342279434204, 0.08424930154766841, -0.016198093071579933, -2.9593945942320765, -2.299056355153219, 0.2700521647930145], [0.7682886719703674, -0.06335695207629399, -0.016745924949645996, -2.6627713642516078, -2.3000953833209437, 0.2357138693332672], [0.7319808602333069, -0.21612353742633061, -0.01674938201904297, -2.6632024250426234, -2.781879965459005, 0.23604820668697357], [0.7048665285110474, -0.24396570146594243, -0.01671004295349121, -2.6394559345641078, -2.7726085821734827, 0.23595722019672394], [0.6371080875396729, -0.23979349554095464, -0.016338340938091278, -2.1704617939391078, -2.755148712788717, 0.2360275685787201], [0.6389087438583374, -0.23969884336505132, -0.016305796802043915, -1.774459024468893, -2.7260170618640345, 0.23600031435489655], [0.6484822630882263, -0.2503926318934937, -0.016533762216567993, -1.3356142503074189, -2.6926429907428187, 0.23599670827388763], [0.660761296749115, -0.5104695123485108, -0.016792330890893936, -1.2909068626216431, -2.6898966471301478, 0.23594528436660767], [0.6614895462989807, -0.6573170584491272, -0.016809366643428802, -1.291806475525238, -2.6902156511889856, 0.23604464530944824], [0.6026541590690613, -0.7954316300204773, -0.016884852200746536, -1.3146988314441224, -2.6908846537219446, 0.23668529093265533], [0.4455782175064087, -0.9123553794673462, -0.016853921115398407, -1.3143952649882813, -2.5968571344958704, 0.2382567971944809], [0.18302752077579498, -0.9610344928554078, -0.01682140678167343, -1.3143111032298584, -2.1112211386310022, 0.238127201795578], [-0.0603869597064417, -0.9607105416110535, -0.0166330449283123, -1.3141909402659913, -1.8285020033465784, 0.23767614364624023], [-0.2374818960772913, -0.9498932522586365, -0.01637434959411621, -1.3108329337886353, -1.8237822691546839, 0.23746013641357422], [-0.3252032438861292, -0.9677493137172242, -0.01673749089241028, -1.312249557381012, -1.7121990362750452, 0.2375476062297821], [-0.3382003943072718, -1.1389659506133576, -0.016793854534626007, -1.3501444619945069, -1.5958903471576136, 0.2375199794769287], [-0.29460078874696904, -1.3111546796611329, -0.0168144591152668, -1.3500972104123612, -1.4846051375018519, 0.23756304383277893], [-0.07905131975282842, -1.317069725399353, -0.016788827255368233, -1.3501804631999512, -1.298185173665182, 0.23756790161132812], [0.2010159194469452, -1.3163175147822876, -0.016602158546447754, -1.3365225356868287, -1.2984808127032679, 0.2375524342060089], [0.45822399854660034, -1.228885905151703, -0.016263063997030258, -1.268704966907837, -1.3307741324054163, 0.2449333667755127], [0.7058058977127075, -1.078401343231537, -0.01633487455546856, -1.7524849377074183, -1.7822273413287562, 0.24345333874225616], [0.9955281615257263, -0.9036885064891358, -0.016329888254404068, -2.1509748897948207, -2.17505914369692, 0.23589229583740234], [1.2307521104812622, -0.700256959801056, -0.01629042625427246, -2.4484902820982875, -2.6441214720355433, 0.23563553392887115], [1.3579720258712769, -0.5471093219569703, -0.016233885660767555, -2.494840761224264, -2.9484856764422815, 0.23569679260253906], [1.3599085807800293, -0.5397361081889649, -0.016338348388671875, -2.3345529041686, -2.606213156376974, 0.23569679260253906], [1.2298200130462646, -0.6739033621600647, -0.016334887593984604, -1.6183468304076136, -2.071836773549215, 0.24037526547908783], [0.9534805417060852, -0.6831450027278443, -0.016377797350287437, -1.2486139696887513, -2.0370634237872522, 0.27487912774086], [0.7216267585754395, -0.7577439707568665, -0.016413824632763863, -1.2484864753535767, -2.1693366209613245, 0.2836764454841614], [0.6326494812965393, -0.869138316517212, -0.016417261213064194, -1.2186070841601868, -2.198613945637838, 0.2833651900291443], [0.5501609444618225, -0.9460297387889405, -0.016497721895575523, -0.9834320706180115, -2.0044291655169886, 0.28328126668930054], [0.4297117292881012, -1.021933690910675, -0.01645137183368206, -1.2051020425609131, -2.002371136342184, 0.28296592831611633], [0.18022704124450684, -1.0251467388919373, -0.01626303605735302, -1.1918645662120362, -1.8505290190326136, 0.28295770287513733], [-0.07433683076967412, -0.9506986302188416, -0.01590680330991745, -1.191744403248169, -1.6541207472430628, 0.28054943680763245], [-0.19261199632753545, -0.8621567052653809, -0.015742531046271324, -0.943945900802948, -1.6555736700641077, 0.2806694209575653], [-0.14625054994692022, -0.8451070946506043, -0.015735682100057602, -0.912394718532898, -1.6178467909442347, 0.281012624502182], [0.04259027913212776, -0.8229121726802369, -0.015846841037273407, -0.9538176220706482, -1.6064532438861292, 0.2811131179332733], [0.2693169414997101, -0.8750217121890564, -0.016239071264863014, -1.2966969174197693, -1.8942011038409632, 0.2811369001865387], [0.2823314368724823, -1.1149813395789643, -0.016441212967038155, -1.3061621946147461, -1.9864586035357874, 0.28145578503608704], [0.32484757900238037, -1.2991897624782105, -0.016415489837527275, -1.280770168905594, -1.985842529927389, 0.2813839912414551], [0.5905853509902954, -1.2830203038505097, -0.015911772847175598, -1.2172344487956543, -2.038278881703512, 0.2814762592315674], [0.8479070067405701, -1.1147517722896119, -0.01590167172253132, -1.3946561229280015, -2.329444948826925, 0.2839090824127197], [1.0485910177230835, -0.9546607297709961, -0.015910297632217407, -1.557668719706573, -2.8167129198657435, 0.29532819986343384], [1.0838836431503296, -0.7384059590152283, -0.015951264649629593, -1.7892300091185511, -2.939153258000509, 0.4193166196346283], [1.0663484334945679, -0.5972893399051209, -0.015867341309785843, -1.7892538509764613, -2.9392851034747522, 0.4434414505958557], [0.9405581951141357, -0.554501013164856, -0.015951285138726234, -1.587973257104391, -2.4468253294574183, 0.7357266545295715], [0.7779669165611267, -0.6488906902125855, -0.01613461971282959, -1.3513524395278473, -2.2114980856524866, 0.8147585391998291], [0.5948438048362732, -0.7026487153819581, -0.016030149534344673, -1.2163091462901612, -2.084717575703756, 0.8142946362495422], [0.5681144595146179, -0.6838953059962769, -0.015778398141264915, -1.17860563219104, -1.9095075766192835, 0.8141951560974121], [0.6644318103790283, -0.5740228456309815, -0.015739083290100098, -1.189978079204895, -1.9911745230304163, 0.8143150806427002], [0.7886213660240173, -0.46943850935015874, -0.015708068385720253, -1.4608229485205193, -2.52091890970339, 0.8142790794372559], [0.8881017565727234, -0.29691369951281743, -0.015691041946411133, -1.815770765344137, -2.8094566504107874, 0.8142780661582947], [0.8890202641487122, -0.12825389326129155, -0.01570655032992363, -2.1588641605772914, -2.8089011351214808, 0.8142347931861877], [0.8855827450752258, -0.03226705015216069, -0.015672212466597557, -2.335623880425924, -2.5717302004443567, 0.8138154149055481], [0.7975271344184875, -0.020359591846801806, -0.015723595395684242, -2.335376878777975, -2.292746607457296, 0.6790518760681152], [0.725882351398468, 0.037768765086791944, -0.015694474801421165, -2.654599805871481, -2.285959307347433, 0.31453806161880493], [0.7173433899879456, 0.09152929365124507, -0.01567912846803665, -2.6464268169798792, -2.276428524647848, 0.30332517623901367], [0.7174153327941895, 0.10110847532238765, -0.015891145914793015, -2.632519384423727, -2.275640312825338, 0.303708553314209], [0.7175629734992981, 0.1011923986622314, -0.015783417969942093, -2.3629399738707484, -2.2755120436297815, 0.30366918444633484], [0.718572735786438, 0.1009179788776855, -0.015906818211078644, -1.8877388439574183, -2.2751320044146937, 0.3036451041698456], [0.7185521125793457, 0.10095278799023433, -0.015902021899819374, -1.879068513909811, -2.2732413450824183, 0.3036331832408905], [0.718543529510498, 0.10101978361096187, -0.015827691182494164, -1.8787600002684535, -2.2701261679278772, 0.3036523163318634], [0.7185056209564209, 0.1010400491901855, -0.015803737565875053, -1.8787404499449671, -2.2701380888568323, 0.3036367893218994], [1.3269356489181519, -0.2141832870296021, -1.393172025680542, -1.5033414301327248, -2.0650299231158655, 0.303653359413147], [1.3256845474243164, -0.20641549051318364, -1.3929800987243652, -1.448477567439415, -2.305425469075338, 0.30365338921546936], [1.2782894372940063, -0.18536074579272466, -1.3854432106018066, -1.384065495138504, -2.305429760609762, 0.30366063117980957], [1.2303038835525513, -0.09661348283801274, -1.3819847106933594, -1.07443060100589, -2.3060501257525843, 0.304036021232605], [1.0161151885986328, 0.0004586416431884288, -1.1966056823730469, -1.197909103041031, -2.3081613222705286, 0.30407169461250305], [0.9430704116821289, 0.11818425237622066, -0.9394869804382324, -1.9063626728453578, -2.5048895517932337, 0.3039804697036743], [0.9309704303741455, 0.27452556669201655, -0.8943867087364197, -2.271248003045553, -2.5273199717151087, 0.303661972284317], [0.9313729405403137, 0.2745207983204345, -0.8111952543258667, -2.3122688732542933, -2.452292267476217, 0.3036569654941559], [0.9281248450279236, 0.15678708135571284, -0.8139103651046753, -2.3503204784789027, -2.4733670393573206, 0.30359721183776855], [0.9301455616950989, 0.11241344987835689, -0.8007464408874512, -2.304445882836813, -2.5446093718158167, 0.3021385669708252], [0.9222211837768555, 0.11307720720257564, -0.7084401249885559, -2.027382036248678, -2.4996045271502894, 0.3028852939605713], [0.8984333276748657, 0.1133841711231689, -0.4544993042945862, -2.332329889337057, -2.518136803303854, 0.30730438232421875], [0.8966448307037354, 0.12330452978100581, -0.32341301441192627, -2.4210034809508265, -2.551687304173605, 0.30726003646850586], [0.9022253751754761, 0.20852450906719966, -0.30143821239471436, -2.5003677807249964, -2.5504513422595423, 0.30724453926086426], [0.9014054536819458, 0.24496404706921382, -0.30007028579711914, -2.7331634960570277, -2.5499287287341517, 0.30563807487487793], [0.9025693535804749, 0.2216450411030273, -0.3062565326690674, -2.7302242718138636, -2.5479865709887903, 0.3050389289855957], [0.9028724431991577, -0.013954953556396532, -0.3943331837654114, -2.7282406292357386, -2.547154490147726, 0.30501842498779297], [0.9227235317230225, -0.11070914686236577, -0.41758251190185547, -2.7087766132750453, -2.542004172001974, 0.3050353527069092], [0.9247246980667114, -0.22285981596026616, -0.5618934035301208, -2.212315698663229, -2.5421398321734827, 0.30511927604675293], [0.922791600227356, -0.22309692323718266, -0.9230064749717712, -1.4606653240374108, -2.541879479085104, 0.3051106929779053], [0.9227111339569092, -0.23026104391131597, -1.1075661182403564, -0.6665178102305909, -2.539541784917013, 0.3050868511199951], [0.9217283725738525, -0.32165010393176274, -1.1227846145629883, -0.2610094112208863, -2.511346165333883, 0.30507969856262207], [0.9216888546943665, -0.5189952415278931, -1.1227846145629883, -0.26028116167102056, -2.5112178961383265, 0.305076003074646], [0.9217009544372559, -0.6195144218257447, -1.1266926527023315, -0.2846685212901612, -2.5108497778521937, 0.30505192279815674], [0.9212591052055359, -0.6288372439197083, -1.1290924549102783, -0.6962937277606507, -2.5138512293445032, 0.3051142990589142], [0.8742664456367493, -0.6564317506602784, -1.1305267810821533, -0.7855299276164551, -2.916934315358297, 0.3051023781299591], [0.615426778793335, -0.7496782702258606, -1.1313505172729492, -0.7339839500239869, -2.7390201727496546, 0.30502206087112427], [0.2977926433086395, -0.8229648631862183, -1.1306875944137573, -0.40091104925189214, -2.54216438928713, 0.30563807487487793], [-0.006926361714498341, -0.7800991100123902, -1.1040250062942505, -0.32935436189685063, -2.2857075373279017, 0.3057028353214264], [-0.21029168764223272, -0.6635811489871521, -1.06499183177948, -0.5242263835719605, -2.0405553022967737, 0.29662659764289856], [-0.2399371306048792, -0.6564228695682068, -1.0645790100097656, -0.7007059019855042, -1.8740080038653772, 0.2935822308063507], [-0.19253188768495733, -0.8282238405993958, -1.0649592876434326, -0.8332073849490662, -1.7570646444903772, 0.02132071740925312], [-0.07785588899721319, -0.874840037231781, -1.0650672912597656, -0.7181957525065918, -1.3986066023456019, 0.02240525558590889], [0.1963796317577362, -0.8746923965266724, -1.0650913715362549, -0.7300029557994385, -1.3834245840655726, 0.02244226448237896], [0.4255693256855011, -0.8747706574252625, -1.0650948286056519, -1.1380549234202881, -1.383752171193258, 0.02278134785592556], [0.44819629192352295, -0.8747799557498475, -1.0667316913604736, -1.3524208080819626, -1.3839595953570765, 0.022793184965848923], [0.4481325149536133, -0.8745593589595337, -1.0667747259140015, -1.8066135845580042, -1.3839605490313929, 0.022748947143554688], [0.5713692903518677, -0.8559327882579346, -1.0667747259140015, -1.8059412441649378, -1.3839815298663538, 0.022748947143554688], [0.9080055356025696, -0.7870509785464783, -1.0665709972381592, -1.8058296642699183, -1.3839886824237269, 0.02274198830127716], [1.0400395393371582, -0.62064482391391, -1.066294550895691, -1.805349966088766, -1.3839572111712855, 0.02276463992893696], [1.1277822256088257, -0.4844523233226319, -1.0662777423858643, -1.715433736840719, -1.3840282599078577, 0.02274196408689022], [1.386447787284851, -0.4703691762736817, -1.0661952495574951, -1.3463903826526185, -1.3840602079974573, 0.022773027420043945], [1.5440016984939575, -0.4472084802440186, -1.0603214502334595, -1.3465057176402588, -1.384024445210592, 0.022769581526517868], [1.4664093255996704, -0.3265530628016968, -1.0552077293395996, -1.3464615058949967, -1.3840363661395472, 0.022773034870624542], [1.3151178359985352, -0.2211488050273438, -1.0551296472549438, -1.345689134006836, -1.384024445210592, 0.022773027420043945], [1.07943856716156, -0.16838152826342778, -1.055346965789795, -1.200525091295578, -1.3840444723712366, 0.02275766059756279], [0.8120314478874207, -0.3405662339976807, -1.0555475950241089, -0.8564868134311219, -1.3841479460345667, 0.022773034870624542], [0.5782783031463623, -0.4878750604442139, -1.0556867122650146, -0.9487337034991761, -1.3840964476214808, 0.022754216566681862], [0.5135061144828796, -0.4859505456737061, -1.0553314685821533, -0.9494119447520752, -1.386143986378805, 0.02274543233215809], [0.5138809084892273, -0.36087055624041753, -1.055182695388794, -0.9490402501872559, -1.7684267202960413, 0.022748947143554688], [0.5133548974990845, -0.29273636758837895, -1.0551360845565796, -0.9314447802356263, -2.1229680220233362, 0.022713184356689453], [0.5125739574432373, -0.29218001783404546, -1.0314444303512573, -0.6313154858401795, -2.144606892262594, -0.1697147528277796], [0.5095673203468323, -0.32436247289691167, -0.8566430807113647, -0.7645874184421082, -2.1534550825702112, -0.4468596617328089], [0.5547515749931335, -0.39849718034777837, -0.7832701206207275, -0.8894392412951966, -2.152163330708639, -0.4794815222369593], [0.6174567937850952, -0.5375107091716309, -0.7818641662597656, -0.9778692883304139, -2.153515164052145, -0.4880431334124964], [0.6175685524940491, -0.6985963743976136, -0.7843313217163086, -1.2674025160125275, -2.1538546721087855, -0.07341891924013311], [0.6173655390739441, -0.7116752427867432, -0.7842304706573486, -1.2922192376903077, -2.1539023558246058, 0.017117738723754883], [0.6171411275863647, -0.7139614981463929, -0.7840661406517029, -1.2513652008822937, -2.1534350554095667, 0.01710556447505951], [0.6171374917030334, -0.7510704559138794, -0.7841464877128601, -0.7145197552493592, -2.1522963682757776, 0.017093658447265625], [0.6160283088684082, -0.9249503177455445, -0.7984107136726379, -0.037470774059631395, -2.151956383381979, 0.017095286399126053], [0.6110872626304626, -1.0054668349078675, -0.8187468647956848, -0.10039790094409184, -2.151243511830465, 0.017090225592255592], [0.6040946245193481, -1.0343110126307984, -1.0982582569122314, -0.09945650518450933, -2.150832001362936, 0.017097104340791702], [0.6041833758354187, -1.037090615635254, -1.1622344255447388, -0.09134419382128911, -2.1508801619159144, 0.017105579376220703], [0.6046494841575623, -1.0395534199527283, -1.1621145009994507, -0.11672003686938481, -2.151167694722311, 0.017033830285072327], [0.6047031283378601, -0.9861452144435425, -1.1619960069656372, -0.7521190208247681, -2.173610035573141, 0.01683194376528263], [0.6395770311355591, -0.8795584005168458, -1.1621387004852295, -1.1554460686496277, -2.1718266646014612, 0.016841888427734375], [0.8647984266281128, -0.6596787136844178, -1.1619001626968384, -1.1694194835475464, -2.171391312276022, 0.01685757376253605], [0.9986358284950256, -0.557617263203003, -1.1450313329696655, -1.1935704511455079, -2.171483341847555, 0.01683144085109234], [0.9973977208137512, -0.5566108983806153, -0.9028922915458679, -1.1935547453216095, -2.1713793913470667, 0.016897056251764297], [0.9976878762245178, -0.5099353355220337, -0.5402538180351257, -1.1933626693538208, -2.166694943104879, 0.01687788963317871], [0.9941394925117493, -0.48047693193469243, -0.39817166328430176, -1.1933458310416718, -1.7220996061908167, 0.0168540608137846], [0.9906511306762695, -0.46212859571490483, -0.39790773391723633, -1.1921768945506592, -1.6667855421649378, 0.016865968704223633], [0.9906872510910034, -0.46186240137133794, -0.39789581298828125, -1.1921563011458893, -1.6667783896075647, 0.01686248742043972], [0.9911751747131348, -0.4617305558970948, -0.33788424730300903, -1.192197398548462, -1.6668217817889612, 0.01685054786503315], [0.9911536574363708, -0.45710845411334233, 0.07462865511049444, -1.192288414841034, -1.6667936483966272, 0.016841888427734375], [0.9842712879180908, -0.4259947103312989, 0.4410894552813929, -1.1924805802157898, -1.6667855421649378, 0.01686946116387844], [0.9450486302375793, -0.4112047714046021, 0.575179402028219, -1.195066974764206, -1.666844669972555, 0.01685054786503315], [0.8303588628768921, -0.41100485742602544, 0.5752242247210901, -1.2298838061145325, -1.6773222128497522, 0.016838299110531807], [0.8060046434402466, -0.39717920244250493, 0.5753472487079065, -1.991094251672262, -1.9773786703692835, 0.01687788963317871], [0.8064231276512146, -0.30752475679431157, 0.5753644148456019, -2.4373446903624476, -2.1486128012286585, 0.016901962459087372], [0.8044697046279907, -0.2759825748256226, 0.5751522223102015, -2.9481221638121546, -2.1972535292254847, 0.016857627779245377], [0.8303391337394714, -0.2705133718303223, 0.5796893278705042, -2.9145456753172816, -2.1956451574908655, 0.01681804656982422], [0.8307514786720276, -0.27866263807330327, 0.5806892553912562, -2.6970936260619105, -2.194954220448629, 0.016821570694446564], [0.8269815444946289, -0.39970441282305913, 0.5813944975482386, -1.897942682305807, -2.1926825682269495, 0.016813253983855247], [0.8269454836845398, -0.41956789911303716, 0.5931909720050257, -1.5737973652281703, -2.192418877278463, 0.01680612564086914], [0.8263459801673889, -0.4195149701884766, 0.8917177359210413, -1.5458672915450116, -2.1923792997943323, 0.016809724271297455], [0.8269226551055908, -0.41933877885852056, 0.9246304670916956, -1.5459858451834698, -2.192383114491598, 0.016790339723229408], [0.8189169764518738, -0.4158197206309815, 0.9246662298785608, -1.822869440118307, -2.192406956349508, 0.01681450568139553], [0.8074743747711182, -0.41553910196337895, 0.9246271292315882, -2.670385023156637, -2.1925986448871058, 0.016829952597618103], [0.7630266547203064, -0.3853294414332886, 0.9246786276446741, -3.0758615932860316, -2.18885308900942, 0.016774892807006836], [0.7691527605056763, -0.37700314939532475, 0.9247778097735804, -3.0242353878416957, -2.186688248311178, 0.016782045364379883], [0.7654032111167908, -0.33658678949389653, 0.9232681433307093, -3.2217966518797816, -2.1289356390582483, 0.01684190332889557], [0.7607702016830444, -0.3288244766047974, 0.9101312796222132, -3.4074093304076136, -1.9726327101336878, 0.016849007457494736], [0.7205820679664612, -0.3309646409801026, 0.8643983046161097, -3.40911926845693, -1.9732840696917933, 0.016841888427734375], [0.6512686014175415, -0.3068465751460572, 0.8644517103778284, -3.408656259576315, -1.9730270544635218, 0.016829967498779297], [0.651904821395874, -0.28684289873156743, 0.862835709248678, -3.388956209222311, -1.973044220601217, 0.016782045364379883], [0.6525014042854309, -0.28656669080767827, 0.8628152052508753, -3.045511385003561, -1.9730361143695276, 0.016837095841765404], [0.6874575018882751, -0.3340976995280762, 0.8549426237689417, -2.771280904809469, -1.972971264516012, 0.016825245693325996], [0.7485769391059875, -0.44524438798937993, 0.7839544455157679, -2.283926626245016, -1.973004166279928, 0.016774863004684448], [0.7538490891456604, -0.4748760026744385, 0.5816944281207483, -2.208653589288229, -1.973459545766012, 0.016802560538053513], [0.7537447214126587, -0.4744911950877686, 0.5774505774127405, -2.3201595745482386, -1.9733198324786585, 0.016809780150651932], [0.7593370676040649, -0.5001802009395142, 0.5772860685931605, -2.444197794000143, -1.97337514558901, 0.016790343448519707], [0.7665926218032837, -0.6734681886485596, 0.5743935743915003, -2.580637594262594, -1.973360840474264, 0.0168299600481987], [0.7668076157569885, -0.8643360298923035, 0.5736058394061487, -2.578557630578512, -1.9733765761004847, 0.016798997297883034], [0.7673765420913696, -0.8742041748813172, 0.5736058394061487, -2.5753780804076136, -1.973360840474264, 0.01680612564086914], [0.7671279311180115, -0.8748112481883545, 0.5329230467425745, -2.5754219494261683, -1.9733641783343714, 0.016821617260575294], [0.7678205370903015, -0.8745114368251343, 0.5209820906268519, -2.574958940545553, -1.973419491444723, 0.01681804656982422], [0.7929733991622925, -0.8417331141284485, 0.6242530981646937, -2.575215002099508, -1.9733236471759241, 0.016790319234132767], [0.8092496991157532, -0.6874604982188721, 0.6254161039935511, -2.5750748119749964, -1.9729960600482386, 0.01680612564086914], [0.8148627877235413, -0.5610093635371705, 0.6266863981830042, -2.5739327869811, -1.9729960600482386, 0.016790330410003662], [0.8140400648117065, -0.43170078218493657, 0.6267297903644007, -2.5568486652769984, -1.9729722181903284, 0.01681804656982422], [0.8120999932289124, -0.40203674257312016, 0.6267154852496546, -2.4181329212584437, -1.9729998747455042, 0.016778454184532166], [0.8004451990127563, -0.3459403079799195, 0.6267073790179651, -2.10886349300527, -1.9727962652789515, 0.01680612564086914], [0.7940670847892761, -0.2887326043895264, 0.6266182104693812, -2.6161533794798792, -1.9654629866229456, 0.016802560538053513], [0.7603307366371155, -0.28347452104602056, 0.6265304724322718, -2.858049055139059, -1.9076264540301722, 0.01681804656982422], [0.7605384588241577, -0.2829628747752686, 0.6266744772540491, -2.8471790752806605, -1.9082005659686487, 0.016821611672639847], [0.7581987977027893, -0.23090334356341558, 0.6267226378070276, -2.8448969326415003, -1.98784047762026, 0.01681448146700859], [0.7405150532722473, -0.22531028211627202, 0.6055014769183558, -2.877929826776022, -2.270043675099508, 0.01678316667675972], [0.7247697114944458, -0.20809419572863774, 0.6055887381183069, -2.878353258172506, -2.303214136754171, 0.016785597428679466], [0.7230772972106934, -0.20864565790209966, 0.6033085028277796, -2.8877755604186, -2.3144694010363978, 0.01680612564086914], [0.7235573530197144, -0.2319200795939942, 0.5581525007831019, -2.887707849542135, -2.3086326758014124, 0.01681448146700859], [0.7289706468582153, -0.23190327108416753, 0.5447915236102503, -2.8879553280272425, -2.162727181111471, 0.01687788963317871], [0.7290546894073486, -0.26304514825854497, 0.5179055372821253, -2.8846427402892054, -2.1620400587665003, 0.016810918226838112], [0.7293992042541504, -0.22774393976245122, 0.5192363897906702, -2.874996324578756, -2.1621034781085413, 0.01686481200158596], [0.7596663236618042, -0.21963079393420415, 0.5185015837298792, -2.8738204441466273, -2.162015740071432, 0.016834681853652], [0.7261672019958496, -0.3651425403407593, 0.5070136229144495, -2.8740369282164515, -2.2607505957232874, 0.016742408275604248], [0.7175522446632385, -0.4500362438014527, 0.5068658033954065, -2.8739010296263636, -2.3959627787219446, 0.014241933822631836], [0.7053232192993164, -0.6424577993205567, 0.5066025892840784, -2.872993608514303, -2.3971052805529993, 0.014201202429831028], [0.6671979427337646, -0.7734332245639344, 0.5065782705890101, -2.873045583764547, -2.3966413179980677, 0.014201132580637932], [0.6379915475845337, -0.9293870490840455, 0.5065262953387659, -2.873589178124899, -2.3969460169421595, 0.014209473505616188], [0.7364298701286316, -1.1675716501525422, 0.5064981619464319, -2.873693128625387, -2.396630350743429, 0.014178549870848656], [0.9004332423210144, -1.2104474765113373, 0.506510082875387, -2.873629232446188, -2.3966859022723597, 0.01422489620745182], [1.0637240409851074, -1.2108108562282105, 0.506502930318014, -2.873624940911764, -2.396670166646139, 0.014221394434571266], [1.2068508863449097, -1.2107555729201813, 0.5065501371966761, -2.873545309106344, -2.396689478551046, 0.01419874932616949], [1.3501660823822021, -1.1974352461150666, 0.5112598578082483, -2.873045583764547, -2.3966978232013147, 0.014221394434571266], [1.4215312004089355, -1.113056705599167, 0.5260456244098108, -2.87304224590444, -2.3966618219958704, 0.0142059326171875], [1.5411617755889893, -1.0221989315799256, 0.5260904471026819, -2.8729664287962855, -2.396637741719381, 0.014209473505616188], [1.5879340171813965, -0.9243976038745423, 0.5261371771441858, -2.8728743992247523, -2.3965898195849817, 0.0142059326171875], [1.5922365188598633, -0.8480584782413025, 0.5261691252337855, -2.8727904758849085, -2.396562401448385, 0.014174985699355602], [1.583549976348877, -0.7710682314685364, 0.5261810461627405, -2.865984102288717, -2.3965778986560267, 0.014205940067768097], [1.4183237552642822, -0.7340181034854432, 0.5261252562152308, -2.857072492639059, -2.3964579741107386, 0.014194011688232422], [1.1514338254928589, -0.7442791026881714, 0.5255101362811487, -2.851274629632467, -2.3962181250201624, 0.014205925166606903], [1.0028499364852905, -0.9094963234714051, 0.5245230833636683, -2.807891984979147, -2.396302048360006, 0.0142059326171875], [0.999713659286499, -1.1448391538909455, 0.52441913286318, -2.8080855808653773, -2.3963785807238978, 0.014190459623932838], [0.9997220039367676, -1.3190131348422547, 0.5243914763080042, -2.80822815517568, -2.3964975515948694, 0.01421422977000475]]