#include <gnc_functions.hpp>
#include <math.h>
//include API 


double xList [500] = {
115.814152461017,116.950675759959,117.802203133305,118.894146173780,120.415240698807,122.240825041580,124.221167692361,126.201117281175,128.191059324017,130.058083581566,131.948667163920,133.863673727273,135.803184601255,137.765933880686,139.748110839941,141.600103646864,143.490779166965,145.418715167241,147.155880406820,148.950636132024,150.801715890186,152.704824395087,154.405852818684,156.174711364196,158.008029116042,159.899033705891,161.583876777259,163.340696866763,165.165042269793,167.049817459525,168.985425816117,170.754115917375,172.591035035602,174.489044540348,176.189054316566,177.964829290098,179.810076735087,181.423400647219,183.119118421684,184.893245981286,186.738684964825,188.355397188443,190.057099464140,191.460973991648,192.958757831922,194.553991704244,196.244830700800,198.026121351315,199.554274481099,201.187037399435,202.922543979269,204.390064002207,205.972774616357,207.665768555289,209.459443995652,211.013983427161,212.680586530606,214.058234410587,215.563345307145,217.195348987064,218.539928696107,220.029181855025,221.662097897896,223.429337296064,224.981141125818,226.679550278219,228.139140567433,229.764105261187,231.538256216703,233.115301378185,234.852410587363,236.381201301572,238.084663112478,239.583438552820,241.272579024053,242.754969776577,244.437121063003,246.281999124587,247.992740172229,249.857736348808,251.591437892966,253.470703575485,255.223159944030,257.111764241531,258.874743258714,260.769878997294,262.537605312489,264.117584906880,265.869349112169,267.433931543540,269.169465338206,270.712148359067,272.437612222323,273.978985800318,275.714357198904,277.277122808289,279.026575126433,280.606448629077,282.375351362797,283.988282385788,285.788729583773,287.445920523755,289.277753597584,290.975672337313,292.836383483248,294.573065028053,296.460344948065,298.229140622264,299.828409226621,301.609657764821,303.528750298150,305.461740737502,307.408374205419,309.375297814920,311.359668881767,313.356159756956,315.355889262214,317.344928151127,319.302939603341,321.202186952637,323.007121770331,324.674994035196,326.157661892389,327.404546264818,328.883463028495,330.125115375527,331.599118190783,332.834527016616,334.302397520654,335.529865213098,336.990384477067,338.208845780896,339.661429777862,340.870733593028,342.315679782602,343.516853276426,344.955637596761,346.150523360298,347.584580858864,349.232331976175,350.689371587822,352.357371641816,353.840736965397,355.531514024127,357.045032178006,358.761452037778,360.309109624848,362.054657730935,363.642135255694,365.420682206585,367.339294001695,369.169063487019,371.119527238995,372.999571852029,374.976627078516,376.903266537729,378.898303128927,380.864251616188,382.863779920577,384.856366124279,386.840483127298,388.839835346446,390.780716545021,392.757972156188,394.618470403172,396.533920450114,398.492945022425,400.309757395241,402.187669512071,403.853426822530,405.592685900059,407.400613588652,408.953526116541,410.583786421521,412.288831668721,413.689614946274,415.169777524664,416.728640224733,417.930040459936,419.208730048860,420.566332936676,422.003415981966,423.519557051727,424.662952943118,425.882600793063,427.180219836846,428.556698186438,429.522787138652,430.560212041552,431.671220918261,432.857726115875,434.121240584008,435.462605117871,436.383718437052,437.374648808243,438.437634633840,439.574691528593,440.787539588737,442.077512856615,442.935747640404,443.861266879549,444.856337286975,445.923053006525,447.063268039782,448.278528657001,449.570035820892,450.429706996205,451.355704981376,452.350262859678,453.415470482007,454.553278278401,455.765375327197,457.053100235773,458.417343344089,459.366958556376,460.384594035676,461.472431038280,462.632519897718,463.866737367480,465.176713041387,466.563406379766,468.027188965342,469.567722533144,471.183758064176,472.460247433516,473.811791114623,475.239461703985,476.743791241305,478.324590815485,479.980738886067,481.709882186969,483.508102216902,485.369517660866,487.285474831276,489.244715545153,491.062805277798,492.941433653847,494.871857649394,496.842183645178,498.836776419892,500.835718502842,502.814446284986,504.810897591553,506.808505796311,508.781109090683,510.697051823175,512.687720463806,514.640131332330,516.518953179295,518.283308368455,520.188012944914,521.989695267204,523.642913322866,525.464943182015,527.145289576882,528.634832810173,530.322723853793,531.821264810089,533.516030034556,535.022512139088,536.289412101910,537.779516833012,539.025789353625,540.496862237290,541.719174702907,543.167870441896,544.362344080570,545.785090862158,546.947520285215,548.340286901477,549.465992737854,550.824328035279,551.908396130273,553.227666281189,554.265093822041,555.540559635171,556.526515717351,557.753651093651,559.206901262767,560.407270876601,561.835628823322,563.005011101547,564.404651222455,565.539117739884,566.906735903211,568.002417094265,569.333874569616,570.386710094377,571.678442146559,573.191184711954,574.465509310880,575.962342954451,577.216749841341,578.695805868453,579.928772528384,581.388437177704,582.598219407495,584.037277118255,585.681246848195,587.122803924348,588.769217184874,590.214186537315,591.863800059887,593.313507520334,594.967901715835,596.425088199189,598.087064393478,599.916383158894,601.608527764093,603.461150156693,605.186870695654,607.064801329351,608.828092999861,610.733154468523,612.719446891054,614.661673012897,616.659705916105,618.633256416612,620.631747955157,622.626357588231,624.608465631803,626.607647309180,628.549349488288,630.528455252297,632.396975673017,634.321280449744,636.073873266275,637.896936517611,639.782703768448,641.465556397212,643.223561035661,645.050937976414,646.637187837718,648.302219396855,650.042606671724,651.496711469415,653.031283541863,654.645202916488,655.921504791687,657.276785348246,658.712001077856,660.227069641944,661.370515599036,662.589566623999,663.886086193236,665.261269252462,666.226014161899,667.260233156344,668.366490682945,669.546933866561,670.803342036950,672.136967057148,673.048183454328,674.026161140010,675.073359977454,676.192007943734,677.384055944259,678.651156514474,679.480233087913,680.371649053243,681.327901181379,682.351376163201,683.444319049403,684.608778775561,685.846615428272,687.159330846844,688.043200314721,688.990442833569,690.003516171525,691.085148465433,692.237483501821,693.462453703670,694.761697851956,696.136350807538,697.098026446230,698.124904607995,699.219167189341,700.382989272233,701.618389087639,702.927151208066,704.310522616618,705.769268405262,707.303467057884,708.912316061661,710.178839143550,711.517741880984,712.930595310382,714.418592686645,715.981910088430,717.619919544835,719.330830003252,721.111460401212,722.956757318524,724.859375363025,726.808833785948,728.607647525008,730.469378503461,732.386445015792,734.347891453430,736.338590685477,738.338461598376,740.322359709889,742.316601017408,744.315367372405,746.291251743700,748.210519872497,750.201359547216,752.151781484657,754.022752773207,755.769047078698,757.657062353374,759.427344717812,761.029653295096,762.805775898188,764.415254018143,766.196005034273,767.810000725473,769.202180495696,770.796141329641,772.162416260471,773.732957176898,775.068770480356,776.611589006089,777.911784546964,779.422061495736,780.680707629562,782.152458378137,783.362707530203,784.789070487758,785.942863977290,787.316611467826,788.406716371931,789.721356496617,790.740934848647,791.989567473417,793.453365333076,794.654277010891,796.073176341439,797.218560706804,798.585304291651,799.667246520527,800.975175274487,801.987244988304,803.230094637483,804.689680885735,805.886687879942,807.303783822283,808.449192716907,809.818599270502,810.907358864676,812.225569517983,813.755117117943,815.041443521509,816.542700293726,817.793543490808,819.263028704157,820.476478983195,821.912576949875,823.548729710048,824.972545567855,826.597733018572,828.007537568744,829.621436256520,831.018813118211,832.623941253855,834.404475793460,836.022923447994,837.814837911596,839.449544450464,841.255168045752,842.910593724491,844.733995118464,846.675925222434,848.534280710633,850.496974990714,852.391385006143,854.372614930823,856.302915050596,858.297961540634,860.280033121019,862.279383494480,864.224392963781,866.206618779809,868.086304927347,870.021778622096,871.997953316275,873.860772016478,875.782335640911,877.531893725621,879.355081184749,880.940954275477,882.610356684043,884.359541872301,885.833556869990,887.393954666790,889.038972771800,890.365493068888,891.778522610046,893.278541650516};
double yList [500] = {
145.500392023006,147.146087860930,148.955756651743,150.631364122590,151.929929271489,152.746777976832,153.026497453265,153.308985728698,153.509312127674,154.226403768766,154.878855849870,155.455700597018,155.943857917670,156.328065239431,156.594475641247,157.349540302787,158.001725918766,158.533762363797,159.524851123898,160.407376980090,161.164677269884,161.779639072417,162.831543204035,163.764891716634,164.564232708181,165.215463574438,166.293101651375,167.248917100673,168.068529112812,168.737575063528,169.240983734384,170.174651408511,170.965681213952,171.596204742374,172.649754342896,173.569875662819,174.341278195069,175.523304402285,176.583748285460,177.507042064417,178.277986263458,179.455373874232,180.506187542247,181.930665099464,183.256049408943,184.462378092427,185.530583825292,186.439981218288,187.730232361257,188.885258444299,189.879248842317,191.238066336419,192.460779349294,193.525566427888,194.410285826703,195.668620899679,196.774258315648,198.224115659934,199.541173437355,200.697272044191,202.177848082776,203.512810639798,204.667620233166,205.604030885371,206.865738012228,207.921866011593,209.289197788314,210.455169376327,211.378418198174,212.608429807324,213.599616770207,214.889112389489,215.937071290775,217.261334172317,218.332223741060,219.674802475001,220.756637513062,221.528923041791,222.564956375190,223.287306194424,224.284441569276,224.968810252447,225.932603112115,226.590762520969,227.535169734500,228.174278905423,229.109770046252,230.336010023298,231.301060368652,232.546886125106,233.540829004538,234.813671923813,235.824995045568,237.099423346046,238.093649733228,239.341753755385,240.310988807651,241.537365469058,242.470630232475,243.653192499366,244.524048335464,245.643743913933,246.446481473553,247.503397720604,248.236714863042,249.228650955708,249.890598463836,250.824066118839,252.025040694389,252.934520570561,253.497622558233,254.010991783423,254.469923305095,254.832154604665,255.081697124209,255.200120877261,255.167228540724,254.958124981712,254.550459490166,253.923674434352,253.062158152511,251.958436218283,250.616163509313,249.052424604702,247.706020056985,246.138123552394,244.786341136241,243.213520411221,241.855081533916,240.276055548061,238.909716102836,237.323729958170,235.948957347242,234.355977660421,232.973179587948,231.374060381092,229.984852088733,228.381029152092,226.986941962994,225.853400496880,224.483351019638,223.379822216264,222.038320327784,220.970016561211,219.662628784812,218.636031071497,217.369241431035,216.392992585024,215.176474969704,214.261723230992,213.696985433722,212.889555170121,212.447185243222,211.764959288715,211.462878783846,210.926166203219,210.785350721905,210.417863925627,210.461298473451,210.289251326372,210.540805929449,210.591704886098,211.074385083558,211.375151185548,212.109008305819,212.684378723610,213.087147546985,213.923325041289,214.611399224400,215.718310504788,216.705720177231,217.560937969853,218.821280256983,219.979835969068,221.025216619837,222.452734118618,223.797768966320,225.050744258309,226.649693123925,228.187533471678,229.656177199405,231.047145323031,232.351490467762,233.992418813410,235.577492667249,237.099394378008,238.550362107871,240.301553741134,242.011451273398,243.674478415652,245.284511153219,246.834843759087,248.318333289652,250.093593905231,251.830849499394,253.524974977209,255.170302184209,256.760585046785,258.288972748960,260.095470204229,261.868437811935,263.603325383008,265.295104793471,266.938244981627,268.526684981198,270.053776741905,271.859591092921,273.632308704798,275.367490143930,277.060219514305,278.705027533031,280.295882882929,281.826165392740,283.288642996471,285.048822234104,286.770570773022,288.448846869428,290.078016546965,291.651772321326,293.163051161836,294.604259538614,295.967102354996,297.242545935274,298.420862175697,299.960529311789,301.434751031226,302.835378279380,304.153328446551,305.378511170177,306.499748686632,307.504767813290,308.380213256448,309.111740798630,309.685420257177,310.087136357283,310.920532855986,311.606649044911,312.129586132346,312.472828714697,312.619796952957,312.554754282271,312.263830986243,312.382919946728,312.285137226974,311.955233842585,311.381506166930,311.188533725350,310.754838209500,310.069251987814,309.127418262317,308.517417871810,307.649120221492,306.523566997325,305.698820252942,304.614183911464,303.279545025414,302.206687241330,300.882159022565,299.820193470423,298.504704333743,296.957137054364,295.623125101514,294.058898518067,292.703928197009,291.120908223542,289.742038883889,288.137908986869,286.732280445795,285.104779958419,283.669439875851,282.016325732803,280.548359377680,278.867646325010,277.364474256050,275.654578324280,274.114063163844,272.373979658947,270.794695400736,269.420627008710,267.820904285804,266.420978010588,264.798466042258,263.369827693359,261.722713331564,260.263391379650,258.590225898423,257.097838217111,255.397386113169,253.870484575621,252.562199464935,251.020740150983,249.694282822806,248.136571886167,246.790320323991,245.215584404702,243.848332009741,242.255715622634,240.866790532824,239.727771864391,238.341441025020,237.205957262226,235.823183394053,234.692353952382,233.314548349575,232.190724588529,230.820831325267,229.708250932838,228.899800034827,227.833663723140,227.080145129848,226.069260061501,225.381236417269,224.437413083847,223.828528200950,223.594771035937,223.117531377454,223.028849261459,222.704660393741,222.782323579126,222.635584314967,222.902506969697,222.959713794819,223.439080718633,223.727421445666,224.440605459333,224.985629427858,225.949174143709,226.771634101171,227.437878707657,228.518622446876,229.472257435324,230.285087993561,231.503205958913,232.611208624559,233.596628399820,234.969792351186,236.252402406682,237.433615449369,238.973438011667,240.444225084296,241.837119843235,243.142710608411,244.783604067234,246.369136974032,247.891975444318,249.344170869741,251.096103305572,252.807941811836,254.474133362901,256.088615908565,257.644713112913,259.135164153874,260.915524981771,262.660105059127,264.364034222812,266.021932509058,267.627865746912,269.175268779403,270.995332524269,272.785689353717,274.542271793410,276.260555518987,277.935510995262,279.561559440467,281.132470138503,282.641369813712,284.435464226990,286.196921467597,287.921358306661,289.603639965830,291.238303222869,292.819267487038,294.339782085174,295.792479318500,297.546098423649,299.262350514948,300.936444121371,302.562949012042,304.135776814471,305.648106709744,307.092504004812,308.460736705821,309.743793405385,310.931903199573,312.479778932002,313.965490700715,315.381063089198,316.717425243793,317.964838046869,319.112411582956,320.148165126653,321.058854637881,321.830138818296,322.446616374041,322.893395802053,323.767620677112,324.498344813749,325.068306204811,325.459110340253,325.651766945455,325.629044011227,325.375768897693,325.527431705865,325.457195973280,325.147549479235,324.585046279436,324.393846411333,323.951292159345,323.244562764585,322.269649333243,321.609802136849,320.679157011056,319.482241298417,318.562791191676,317.375533771659,316.465080138269,315.283971375798,313.848062086399,312.640051796770,311.179472181211,309.941166391097,308.452676277490,307.179997607654,305.660296467466,304.349165834003,302.794878244847,301.440644242147,299.848382646200,298.446423761884,296.812789597356,295.359236078476,293.682432184741,292.175209128068,290.454610399697,288.892267105681,287.529440694850,285.930124859223,284.520612642259,282.881071652964,281.420930573041,279.738848228631,278.225797543900,276.500771487757,274.933823897843,273.566487807516,271.964247421112,270.552922125442,268.913398272576,267.455754389121,265.778076664976,264.273975380651,262.985377639129,261.453919385212,260.132470141438,258.571896113616,257.215204104444,255.625380711642,254.233395571600,253.083176398843,251.678630777278,250.512969698734,249.094360827228,247.913119520130,246.482267359209,245.289135485457,244.378258571009,243.203257639293,242.314976965668,241.162703329859,240.302631569981,239.180327768788,238.358617749101,237.880174992549,237.140908545671,236.756420362578,236.115163242973,235.841799043502,235.318404882662,235.177729715738,235.444922998271,235.495894403390,235.961660615485,236.227707254764,236.910920242606,237.414846404789,237.722634608085,238.450581371367,239.005192259519,239.974236380405,240.796420963623,242.015029407752,243.116435599070,244.086152659444,245.437921791577,246.688984825857,247.826488862856,249.323266702555,250.738663295675,252.061517361030};


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	set_mode("GUIDED");

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;


	for (int i = 0; i < 499; i++) {
		nextWayPoint.x = ((xList[i] - xList[0]) / 20);
		nextWayPoint.y = ((yList[i] - yList[0]) / 20);
		nextWayPoint.z = 3;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);


	}

	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(1) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	
	return 0;

}