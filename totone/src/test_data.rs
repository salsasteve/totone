pub fn generate_sine_samples_i16() -> [i16; 1024] {
    let sine_wave_i16: [i16; 1024] = [
        0, 2052, 4097, 6126, 8130, 10103, 12036, 13921, 15752, 17521, 19222, 20846, 22389, 23844,
        25205, 26467, 27625, 28675, 29612, 30433, 31134, 31713, 32167, 32494, 32695, 32766, 32709,
        32524, 32210, 31770, 31206, 30518, 29711, 28787, 27750, 26604, 25354, 24004, 22559, 21026,
        19410, 17718, 15957, 14132, 12253, 10325, 8356, 6355, 4329, 2285, 233, -1819, -3865, -5896,
        -7904, -9881, -11818, -13710, -15547, -17324, -19032, -20666, -22218, -23683, -25055,
        -26329, -27499, -28561, -29511, -30345, -31060, -31653, -32121, -32464, -32678, -32765,
        -32722, -32551, -32252, -31827, -31276, -30603, -29809, -28898, -27874, -26740, -25501,
        -24162, -22728, -21204, -19598, -17914, -16160, -14343, -12469, -10546, -8582, -6584,
        -4560, -2518, -466, 1586, 3633, 5666, 7677, 9658, 11600, 13497, 15341, 17125, 18842, 20484,
        22046, 23521, 24904, 26189, 27372, 28446, 29409, 30257, 30985, 31592, 32075, 32431, 32660,
        32761, 32734, 32577, 32293, 31881, 31345, 30685, 29905, 29007, 27996, 26874, 25647, 24319,
        22895, 21382, 19784, 18109, 16363, 14552, 12684, 10767, 8807, 6812, 4791, 2751, 700, -1353,
        -3401, -5436, -7450, -9434, -11382, -13284, -15135, -16926, -18650, -20301, -21873, -23358,
        -24752, -26049, -27243, -28330, -29306, -30166, -30908, -31529, -32026, -32397, -32641,
        -32756, -32743, -32601, -32331, -31935, -31412, -30766, -30000, -29115, -28116, -27007,
        -25791, -24475, -23062, -21558, -19970, -18303, -16565, -14761, -12899, -10987, -9031,
        -7040, -5022, -2983, -933, 1120, 3169, 5206, 7223, 9211, 11163, 13071, 14927, 16725, 18458,
        20118, 21698, 23194, 24598, 25906, 27112, 28212, 29200, 30074, 30830, 31465, 31976, 32361,
        32620, 32750, 32751, 32624, 32369, 31986, 31478, 30846, 30093, 29222, 28235, 27138, 25935,
        24629, 23227, 21733, 20154, 18496, 16766, 14969, 13113, 11206, 9255, 7268, 5252, 3216,
        1166, -886, -2937, -4975, -6995, -8986, -10943, -12856, -14719, -16524, -18264, -19933,
        -21523, -23029, -24444, -25763, -26980, -28092, -29094, -29981, -30750, -31399, -31924,
        -32324, -32597, -32741, -32757, -32645, -32404, -32036, -31542, -30924, -30184, -29326,
        -28353, -27269, -26077, -24783, -23391, -21908, -20338, -18689, -16966, -15176, -13327,
        -11426, -9479, -7496, -5482, -3448, -1400, 653, 2704, 4745, 6766, 8762, 10723, 12641,
        14510, 16322, 18070, 19747, 21346, 22862, 24288, 25618, 26847, 27971, 28986, 29886, 30669,
        31331, 31871, 32285, 32572, 32731, 32762, 32664, 32438, 32084, 31604, 31000, 30274, 29430,
        28469, 27397, 26217, 24935, 23554, 22081, 20521, 18880, 17165, 15383, 13540, 11644, 9702,
        7723, 5712, 3680, 1633, -420, -2471, -4514, -6538, -8537, -10502, -12426, -14301, -16120,
        -17875, -19560, -21169, -22694, -24130, -25472, -26713, -27849, -28876, -29790, -30586,
        -31262, -31816, -32244, -32546, -32720, -32765, -32682, -32470, -32131, -31665, -31075,
        -30363, -29532, -28584, -27525, -26357, -25085, -23716, -22252, -20702, -19070, -17363,
        -15588, -13752, -11862, -9925, -7949, -5942, -3912, -1866, 186, 2239, 4282, 6309, 8311,
        10280, 12209, 14090, 15916, 17679, 19373, 20990, 22525, 23972, 25324, 26577, 27725, 28765,
        29692, 30501, 31191, 31759, 32202, 32518, 32706, 32766, 32698, 32500, 32176, 31724, 31148,
        30450, 29632, 28698, 27651, 26495, 25235, 23876, 22423, 20882, 19259, 17561, 15793, 13964,
        12079, 10147, 8175, 6172, 4143, 2099, 46, -2006, -4051, -6080, -8085, -10058, -11992,
        -13879, -15711, -17482, -19184, -20810, -22355, -23812, -25175, -26440, -27600, -28653,
        -29592, -30415, -31119, -31701, -32158, -32488, -32691, -32766, -32712, -32529, -32219,
        -31782, -31220, -30535, -29731, -28810, -27775, -26631, -25383, -24035, -22593, -21062,
        -19448, -17758, -15997, -14174, -12296, -10369, -8401, -6401, -4375, -2332, -280, 1773,
        3819, 5850, 7859, 9836, 11775, 13667, 15506, 17284, 18994, 20630, 22184, 23651, 25025,
        26301, 27474, 28539, 29491, 30328, 31045, 31641, 32112, 32457, 32675, 32764, 32725, 32557,
        32261, 31838, 31290, 30619, 29828, 28920, 27898, 26767, 25530, 24193, 22761, 21240, 19635,
        17953, 16201, 14385, 12512, 10590, 8627, 6629, 4606, 2565, 513, -1540, -3587, -5620, -7632,
        -9613, -11557, -13455, -15300, -17085, -18803, -20448, -22011, -23489, -24874, -26161,
        -27346, -28423, -29389, -30239, -30970, -31579, -32065, -32424, -32657, -32760, -32736,
        -32582, -32301, -31892, -31359, -30702, -29924, -29029, -28020, -26901, -25676, -24350,
        -22929, -21417, -19822, -18148, -16403, -14594, -12727, -10811, -8852, -6858, -4837, -2797,
        -746, 1306, 3355, 5390, 7405, 9390, 11338, 13242, 15093, 16886, 18612, 20265, 21838, 23326,
        24721, 26020, 27217, 28306, 29285, 30148, 30893, 31516, 32016, 32390, 32637, 32755, 32745,
        32606, 32339, 31945, 31425, 30782, 30018, 29137, 28140, 27033, 25820, 24506, 23095, 21593,
        20007, 18342, 16605, 14803, 12942, 11031, 9076, 7086, 5068, 3030, 980, -1073, -3123, -5160,
        -7177, -9166, -11119, -13028, -14886, -16685, -18419, -20081, -21663, -23161, -24568,
        -25878, -27086, -28188, -29179, -30056, -30814, -31452, -31966, -32354, -32615, -32748,
        -32753, -32628, -32376, -31996, -31491, -30862, -30111, -29243, -28259, -27165, -25963,
        -24660, -23260, -21768, -20191, -18535, -16806, -15010, -13156, -11250, -9300, -7314,
        -5298, -3262, -1213, 840, 2890, 4929, 6949, 8942, 10899, 12813, 14677, 16484, 18226, 19896,
        21488, 22995, 24413, 25734, 26954, 28068, 29072, 29962, 30734, 31385, 31914, 32316, 32592,
        32739, 32759, 32649, 32411, 32046, 31554, 30939, 30203, 29347, 28377, 27294, 26105, 24813,
        23424, 21942, 20375, 18727, 17006, 15218, 13370, 11469, 9524, 7541, 5528, 3494, 1446, -606,
        -2658, -4698, -6721, -8717, -10678, -12598, -14468, -16282, -18031, -19710, -21311, -22828,
        -24256, -25589, -26821, -27947, -28964, -29867, -30652, -31318, -31860, -32277, -32567,
        -32729, -32763, -32668, -32444, -32093, -31617, -31015, -30292, -29450, -28493, -27423,
        -26245, -24965, -23586, -22115, -20557, -18918, -17205, -15424, -13582, -11688, -9747,
        -7768, -5758, -3726, -1679, 373, 2425, 4467, 6492, 8491, 10457, 12382, 14259, 16079, 17836,
        19523, 21133, 22660, 24099, 25442, 26686, 27825, 28854, 29770, 30569, 31248, 31804, 32236,
        32540, 32717, 32765, 32685, 32476, 32140, 31677, 31090, 30381, 29552, 28607, 27550, 26385,
        25115, 23748, 22287, 20738, 19108, 17403, 15629, 13795, 11905, 9970, 7994, 5988, 3958,
        1912, -140, -2192, -4236, -6263, -8266, -10236, -12166, -14048, -15875, -17640, -19335,
        -20954, -22491, -23940, -25294, -26550, -27701, -28743, -29672, -30484, -31177, -31747,
        -32193, -32512, -32704, -32766, -32701, -32506, -32184, -31736, -31163, -30467, -29652,
        -28720, -27676, -26522, -25265, -23908, -22457, -20918, -19297, -17600, -15834, -14006,
        -12123, -10192, -8221, -6217, -4190, -2145, -93, 1959, 4004, 6034, 8040, 10014, 11949,
        13837, 15670, 17442, 19146, 20774, 22321, 23780, 25145, 26412, 27575, 28630, 29572, 30398,
        31105, 31689, 32149, 32482, 32688, 32766, 32715, 32535, 32227, 31793, 31234, 30552, 29751,
        28832, 27800, 26659, 25413, 24067, 22627, 21097, 19485, 17797, 16038, 14217, 12339, 10413,
        8446, 6446, 4421, 2378, 326, -1726, -3773, -5804, -7813, -9791, -11731, -13625, -15465,
        -17244, -18956, -20593, -22149, -23619, -24995, -26273, -27448, -28516, -29471, -30310,
        -31030, -31629, -32103, -32451, -32671, -32763, -32727, -32562, -32269, -31849, -31304,
        -30636, -29848, -28942, -27923, -26794, -25559, -24225, -22795, -21276, -19673, -17992,
        -16241, -14427, -12555, -10634, -8672, -6675, -4652, -2611, -560, 1493, 3541, 5574, 7586,
        9568, 11513, 13412, 15259, 17046, 18765, 20411, 21977, 23456, 24844, 26133, 27320, 28400,
        29368, 30221, 30955, 31567,
    ];
    sine_wave_i16
}
