/*
解析MPEG-TS

1.从复用的MPEG-TS流中解析出TS包；
2.从TS包中获取PAT及对应的PMT（PSI中的表格）；
3.从而获取特定节目的音视频PID；
4.通过PID筛选出特定音视频相关的TS包，并解析出PES；
5.从PES中读取到PTS/DTS，并从PES中解析出基本码流ES；
6.将ES交给解码器，获得压缩前的原始音视频数据

*/


#include <_types/_uint16_t.h>
#include <_types/_uint32_t.h>
#include <_types/_uint64_t.h>
#include <_types/_uint8_t.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <sys/_types/_int64_t.h>
#include <sys/_types/_size_t.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

using namespace std;

#define TS_PACKET_SIZE 188
#define TS_SYNC_BYTE 0x47

typedef struct {
    unsigned int adaptation_field_length : 8;
	unsigned int discontinuity_indicator : 1;
	unsigned int random_access_indicator : 1;
	unsigned int elementary_stream_priority_indicator : 1;
	unsigned int PCR_flag : 1;
	unsigned int OPCR_flag : 1;
	unsigned int splicing_point_flag : 1;
	unsigned int transport_private_data_flag : 1;
	unsigned int adaptation_field_extension_flag : 1;

	int64_t program_clock_reference_base; // 33-bits
	unsigned int program_clock_reference_extension; // 9-bits

	int64_t original_program_clock_reference_base; // 33-bits
	unsigned int original_program_clock_reference_extension; // 9-bits

	unsigned int splice_countdown : 8;

	unsigned int transport_private_data_length : 8;

	unsigned int adaptation_field_extension_length : 8;
	unsigned int ltw_flag : 1;
	unsigned int piecewise_rate_flag : 1;
	unsigned int seamless_splice_flag : 1;

	unsigned int ltw_valid_flag : 1;
	unsigned int ltw_offset : 15;

	unsigned int piecewise_rate : 22;

	unsigned int Splice_type : 4;
	int64_t DTS_next_AU;
} ts_adaptation_field;


typedef struct {
	uint16_t transport_error_indicator : 1;
	uint16_t payload_unit_start_indicator : 1;
	uint16_t transport_priority : 1;
	uint16_t pid : 13;
	uint8_t transport_scrambling_control : 2;
	uint8_t adaptation_field_control : 2;
	/*
	'00' = 保留 (供未来使用)
	'01' = 无适配域，仅有载荷
	'10' = 仅有适配域
	'11' = 适配域和载荷都存在
	*/
	uint8_t continuity_counter : 4;
    ts_adaptation_field adaptation;
} ts_header;


typedef struct {
	uint64_t adaptation_field_length :8;
	uint64_t discontinuity_indicator : 1;
	uint64_t elementary_stream_priority_indicator : 1;
	uint64_t priority_indicator : 1;
	uint64_t PCR_flag : 1;
	uint64_t OPCR_flag : 1;
	uint64_t splicing_point_flag : 1;
	uint64_t transport_private_data_flag : 1;
	uint64_t adaptation_field_extension_flag : 1;
	uint64_t program_clock_reference_base : 33;
} ts_adaption_field;


typedef struct {
    uint8_t sid;
    uint8_t stream_type;

    int flags;
    int64_t pts;
    int64_t dts;
    vector<char> data;
} ts_packet;


typedef struct {
    uint16_t program_number;
	unsigned stream_type : 8;
	unsigned elementary_PID : 13;
	unsigned ES_info_length : 12;
	unsigned descriptor;
    uint8_t sid;
    uint8_t cc;
    uint32_t len;
    
    // optional packet length
    uint32_t reversed10:2;
    uint32_t PES_scrambling_control:2;
    uint32_t PES_prioriy:1;
    uint32_t data_alignment_indicator:1;
    uint32_t copyright:1;
    uint32_t original_or_copy:1;
    
    // 7个flag
    uint32_t PTS_DTS_flags : 2;
	uint32_t ESCR_flag : 1;
	uint32_t ES_rate_flag : 1;
	uint32_t DSM_trick_mode_flag : 1;
	uint32_t additional_copy_info_flag : 1;
	uint32_t PES_CRC_flag : 1;
	uint32_t PES_extension_flag : 1;

    uint32_t PES_header_data_length:8;
    
    int64_t pts;
    int64_t dts;
    int64_t ESCR_base;
    uint32_t ESCR_extension;
    uint32_t ES_rate;

    ts_packet pkt;
} ts_pes;

typedef struct {
	unsigned int pid;
	unsigned int program_number;
	unsigned int continuity_counter;
	unsigned int version_number;
	unsigned int PCR_PID;
	unsigned int pminfo_len;
	unsigned int stream_count;
	uint32_t service_type;
	char provider[64];
	char name[64];
	vector<ts_pes> pmt_streams;
} ts_pmt;

typedef struct {
	unsigned int tsid;
	unsigned int ver;
	unsigned int continuity_counter;
	unsigned int pmt_count;
	vector<ts_pmt> pat_programs;
} ts_pat;

typedef struct {
	uint8_t cc;
	ts_pat pat;
} ts_demuxer;

enum ETS_PID {
	TS_PID_PAT = 0x00,
	TS_PID_CAT = 0x01,
	TS_PID_TSDT = 0x02,
	TS_PID_IPMP = 0x03,
	// 16-31 Use by DVB metadata
	TS_PID_NIT = 0x010,
	TS_PID_SDT =  0x11
};

enum STREAM_TYPE {
    PSI_STREAM_H264 = 0x1b,
    PSI_STREAM_H265 = 0x24
};

size_t pat_read(ts_pat* pat, const uint8_t* data, size_t bytes);
size_t sdt_read(ts_pat* pat, const uint8_t* data, size_t bytes);
size_t pmt_read(ts_pmt* pmt, const uint8_t* data, size_t bytes);
size_t pes_read_header(ts_pes* pes, const uint8_t* data, size_t bytes);


int ts_packet_h264_h265_filter(ts_packet *pkt, const ts_pes *pes, size_t size) {
    const uint8_t * data , *end;

    printf("pkt->dts=%lld pkt->pts=%lld pkt->cts=%lld, pkt->codecid=%d\n", pes->dts, pes->pts, pes->pts-pes->dts, pes->stream_type);
    return 0;
}


int pes_packet(ts_packet *pkt, const ts_pes *pes, const uint8_t *data, size_t size, int start) {
    if (pes->stream_type == PSI_STREAM_H264 || pes->stream_type == PSI_STREAM_H265) {

        ts_packet_h264_h265_filter(pkt, pes, size);
    }
    else {
    }
    return 0;
}


static uint32_t adaptation_field_read(ts_adaptation_field *adp, const uint8_t *data, size_t bytes) {
    uint32_t i = 0;
    uint32_t j = 0;

    assert(bytes <= TS_PACKET_SIZE);
    adp->adaptation_field_length = data[i++];
    if (adp->adaptation_field_length > 0) {
        adp->discontinuity_indicator = (data[i] >> 7) & 0x01;
        adp->random_access_indicator = (data[i] >> 6) & 0x01;
        adp->elementary_stream_priority_indicator = (data[i] >> 5) & 0x01;
        adp->PCR_flag = (data[i] >> 4) & 0x01;
        adp->OPCR_flag = (data[i] >> 3) & 0x01;
        adp->splicing_point_flag = (data[i] >> 2) & 0x01;
        adp->transport_private_data_flag = (data[i] >> 1) & 0x01;
        adp->adaptation_field_extension_flag = (data[i] >> 0) & 0x01;
        i++;
    }
    if (adp->PCR_flag) {
        adp->program_clock_reference_base = ((uint64_t)data[i] << 25) | ((uint64_t)data[i+1] << 17) | ((uint64_t)data[i+2] << 9) | ((uint64_t)data[i+3] << 1) | ((data[i+4] >> 7) & 0x01);
        adp->program_clock_reference_extension = ((data[i+4] & 0x01) << 8) | data[i+5];
        i += 6;
    }
    if (adp->OPCR_flag) {
        adp->original_program_clock_reference_base = (((uint64_t)data[i]) << 25) | ((uint64_t)data[i+1] << 17) | ((uint64_t)data[i+2] << 9) | ((uint64_t)data[i+3] << 1) | ((data[i+4] >> 7) & 0x01);
        adp->original_program_clock_reference_extension = ((data[i+4] & 0x01) << 1) | data[i+5];
        i += 6;
    }
    if (adp->splicing_point_flag) {
        adp->splice_countdown = data[i++];
    }
    if (adp->transport_private_data_flag) {
        adp->transport_private_data_length = data[i++];
        for (j = 0; j < adp->transport_private_data_length; j++) {
            // transport_private_data
        }
        i += adp->transport_private_data_length;
    }
    if (adp->adaptation_field_extension_flag) {
        adp->adaptation_field_extension_length = data[i++];
        adp->ltw_flag = (data[i] >> 7) & 0x01;
        adp->piecewise_rate_flag = (data[i] >> 6) & 0x01;
        adp->piecewise_rate_flag = (data[i] >> 5) * 0x01;
        
        i++;
        if (adp->ltw_flag) {
            i += 2;
        }
        if (adp->piecewise_rate_flag) {
            i += 3;
        }
        if (adp->seamless_splice_flag) {
            i += 5;
        }
    }
    return adp->adaptation_field_length + 1;   
} 


int ts_demuxer_input(ts_demuxer *ts, const uint8_t* data, size_t bytes) {
	assert(188 == bytes);
	assert(0x47 == data[0]);

	ts_header pkhd;
	uint32_t i;

	uint32_t PID;

	PID = (data[1] << 8 | data[2]) & 0x1FFF;
	memset(&pkhd, 0, sizeof(pkhd));
    memset(&pkhd.adaptation, 0, sizeof(pkhd.adaptation));
	pkhd.transport_error_indicator = (data[1] >> 7) & 0x01;
	pkhd.payload_unit_start_indicator = (data[1] >> 6) & 0x01;
	pkhd.transport_priority = (data[1] >> 5) & 0x01;
	pkhd.transport_scrambling_control = (data[3] >> 6) & 0x03;
	pkhd.adaptation_field_control = (data[3] >> 4) & 0x03;
	pkhd.continuity_counter = (data[3]) & 0x0F;

	if (((ts->cc + 1) % 15) != (uint8_t)pkhd.continuity_counter) {
		// PAT/PMT Reset
	}
	ts->cc = (uint8_t)pkhd.continuity_counter;

	// 打印出每个包的头部信息
	i = 4;
	if (0x02 & pkhd.adaptation_field_control) {
		// 仅有适应域
        i += adaptation_field_read(&pkhd.adaptation, data + i, bytes - i);
        if (pkhd.adaptation.adaptation_field_length > 0 && pkhd.adaptation.PCR_flag) {
            int64_t t;
            t = pkhd.adaptation.program_clock_reference_base / 90L;
            // printf("timebase=%lld\n", t);
        }
	}

	if (0x01 & pkhd.adaptation_field_control) {
		// 仅有负载
		switch (PID)
		{
		case TS_PID_PAT:
			// PAT
			if (pkhd.payload_unit_start_indicator) {
				i += 1;
			}
			pat_read(&ts->pat, data + i, bytes - i);
			break;
		case TS_PID_SDT:
			// SDT
			if (pkhd.payload_unit_start_indicator) {
				i += 1;
			}
			sdt_read(&ts->pat, data + i, bytes - i);
			break;
		default:
			// other data 
			uint32_t j = 0;
			for (; j < ts->pat.pmt_count; j++) {
				if (PID == ts->pat.pat_programs[j].pid) {
					if (pkhd.payload_unit_start_indicator) {
						i += 1;
					}
					pmt_read(&ts->pat.pat_programs[j], data + i, bytes - i);
                    break;
                } else {
                    for (uint32_t k = 0; k < ts->pat.pat_programs[j].stream_count; k++) {
                        ts_pes * pes = &ts->pat.pat_programs[j].pmt_streams[k];
                        if (PID != pes->elementary_PID) {
                            continue;
                        }

                        if (pkhd.payload_unit_start_indicator) {
                            size_t n;
                            n = pes_read_header(pes, data + i, bytes - i);
                            assert(n > 0);
                            i += n;
                        }
                        else if (0 == pes->sid) {
                            continue;
                        }
                        pes_packet(&pes->pkt, pes, data + i, bytes - i, pkhd.payload_unit_start_indicator);
                        break;
                    }      
                }

			}
			break;
		}
	}
	return 0;
}



size_t sdt_read(ts_pat* pat, const uint8_t* data, size_t bytes) {
	uint32_t table_id = data[0];
	uint32_t section_length = ((data[1] & 0x0F) << 8) | data[2];

	if (table_id != 0x02 || section_length + 3 > bytes) {
		return 0;
	}

	uint32_t i = 11, n = 0;
	uint32_t service_id;
	for (; i + 5 <= section_length + 11 - 8/*follow section length*/ - 4; i += 5 + n) {
		n = ((data[i + 3] & 0x0F) << 8) | data[i + 4];
		service_id = (data[i] << 8) | data[i + 1];
		if (i + 5 + n > section_length + 3 - 4) {
			continue;
		}

		uint32_t p = 0;
		for (; p < pat->pmt_count; p++) {
			if (pat->pat_programs[p].program_number == service_id) {
				break;
			}
		}
		// not found the pmt 
		if (p == pat->pmt_count) {
			continue;
		}

		uint32_t k = 0, taglen = 0, tagid = 0;
		for (k = i + 5; k + 2 <= i + 5 + n; k += 2 + taglen) {
			// descriptor_tag
			tagid = data[k];
			taglen = data[k + 1];
			if (0x48 != tagid || k + taglen > i + 5 + n) {
				continue;
			}
			// service type, service provider name , service name
			uint32_t service_type = data[k + 2];
			uint8_t provide_len = data[k + 3];
			if (provide_len >= sizeof(pat->pat_programs[p].provider) || k + 3 + provide_len > i + 5 + n) {
				continue;
			}
			memcpy(pat->pat_programs[p].provider, &data[k + 4], provide_len);
			pat->pat_programs[p].provider[taglen] = 0;
			uint8_t name_len = data[k + 4 + provide_len];
			if (name_len >= sizeof(pat->pat_programs[p].name) || k + 5 + provide_len + name_len > i + 5 + n) {
				continue;
			}
			memcpy(pat->pat_programs[p].name, &data[k + 5 + provide_len], name_len);
			pat->pat_programs[p].name[name_len] = 0;
		}
	}

	return 0;
}


size_t pat_read(ts_pat* pat, const uint8_t* data, size_t bytes) {

	uint32_t table_id = data[0];
	uint32_t section_syntax_indicator = (data[1] >> 7) & 0x01;
	uint32_t section_length = ((data[1] & 0x0F) << 8) | data[2];
	uint32_t transport_stream_id = (data[3] << 8) | data[4];
	uint32_t version_number = (data[5] >> 1) & 0x1F;


	assert(table_id == 0x00);
	assert(section_syntax_indicator == 1);

	if (pat->ver != version_number) {
		pat->pmt_count = 0;
	}

	pat->tsid = transport_stream_id;
	pat->ver = version_number;

	assert(bytes >= section_length + 3);
	uint32_t i;
	uint16_t pn, pid;
	// - 5 fllow section length item  - 4 crc
	for (i = 8; i + 4 <= section_length + 8 - 5 - 4; i += 4)
	{
		pn = (data[i] << 8) | data[i + 1];
		pid = ((data[i + 2] & 0x1F) << 8) | data[i + 3];
		// 打印出来PAT存储的节目信息
		if (pn == 0)
		{
			// 节目号为0x0000 表示这是NIT
			continue;
		}
		// pn = 0x0001 表示这是 PMT
		ts_pmt prg;
		prg.program_number = pn;
		prg.pid = pid;
		pat->pat_programs.push_back(prg);
		pat->pmt_count++;
	}

	// 最好再验证一下CRC32

	return 0;
}


size_t pmt_read(ts_pmt* pmt, const uint8_t* data, size_t bytes) {

	uint8_t table_id = data[0];
	uint32_t section_syntax_indicator = (data[1] >> 7) & 0x01;
	uint32_t section_length = ((data[1] & 0x0F) << 8) | data[2];
	uint16_t program_number = (data[3] << 8) | data[4];
	uint8_t version_number = (data[5] >> 1) & 0x1F;
	uint8_t current_next_indicator = data[5] & 0x01;
	uint32_t sector_number = data[6];
	uint32_t last_sector_number = data[7];
	uint32_t PCR_PID = ((data[8] & 0x1F) << 8) | data[9];
	uint32_t program_info_length = ((data[10] & 0x0F) << 8) | data[11];

	// PMT 表的 table id 一定是 0x02
	assert(0x02 == table_id);
	assert(1 == section_syntax_indicator);

	if (pmt->version_number != version_number) {
		pmt->stream_count = 0;
	}

	pmt->PCR_PID = PCR_PID;
	pmt->program_number = program_number;
	pmt->pminfo_len = program_info_length;
    pmt->version_number = version_number;

	int len = 0;
	int pos = 12;
	if (program_info_length > 0) {
		// descriptor
		
	}
	pos += program_info_length; /* program info length 后面是 descriptor 要跳过 */
	unsigned int pid, stream_type, desc;
	for (; pos + 5 <= section_length + 12 - 9/*section item*/ - 4/*CRC32*/; pos += len + 5) {
		stream_type = data[pos];
		pid = ((data[pos + 1] & 0x1F) << 8) | data[pos + 2];
		len = ((data[pos + 3] & 0x0F) << 8) | data[pos + 4];
        if (pos + len + 5 > section_length + 3 - 4) break;
        desc = 0x00;
		if (len != 0) {
			// descriptor
			// 这里忽略
		}
		ts_pes st;
        st.program_number = (uint16_t)pmt->program_number;
		// 编码器类型
        st.stream_type = stream_type;
		st.elementary_PID = pid;
		st.ES_info_length = 0; // default nothing
		st.descriptor = desc;
        st.sid = 0;
		pmt->pmt_streams.push_back(st);
        pmt->stream_count++;
	}

	return 0;
}

#define DATA(off) data_read_with_bodunary_check(data, bytes, off, &overflow)
static inline uint8_t data_read_with_bodunary_check(const uint8_t *data, size_t bytes, size_t off, int *overflow) {
    return (off < bytes) ? (data[off]) : (*overflow = 1, 0);
}


size_t pes_read_header(ts_pes* pes, const uint8_t* data, size_t bytes) {
    size_t i;
    int overflow;
    
    overflow = 0;
    if (bytes < 3 + 6) return 0;

    // prefix
    assert(0x00 == data[0] && 0x00 == data[1] && 0x01 == data[2]);
    pes->sid = data[3];
    pes->len = (data[4] << 8) | data[5];
    i = 6;
    assert(0x02 == (DATA(i) >> 6 & 0x03));
    pes->PES_scrambling_control = (DATA(i) >> 4) & 0x03;
    pes->PES_prioriy = (DATA(i) >> 3) & 0x01;
    pes->data_alignment_indicator = (DATA(i) >> 2) & 0x01;
    pes->copyright = (DATA(i) >> 1) & 0x01;
    pes->original_or_copy = (DATA(i) & 0x01);
    
    i++;
    pes->PTS_DTS_flags = (DATA(i) >> 6) & 0x03;
    pes->ESCR_flag = (DATA(i) >> 5) & 0x01;
    pes->ES_rate_flag = (DATA(i) >> 4) & 0x01;
    pes->DSM_trick_mode_flag = (DATA(i) >> 3) &0x01;
    pes->additional_copy_info_flag = (DATA(i) >> 2) & 0x01;
    pes->PES_CRC_flag = (DATA(i) >> 1) & 0x01;
    pes->PES_extension_flag = (DATA(i) & 0x01);

    i++;
    pes->PES_header_data_length = DATA(i);

    i++;
    if (0x02 & pes->PTS_DTS_flags) {
        // PTS
        // 判断是否有数据
        assert(0x20 == (DATA(i) & 0x20));
        pes->pts = ((((uint64_t)DATA(i) >> 1) & 0x07) << 30) | ((uint64_t)DATA(i + 1) << 22) | ((((uint64_t)DATA(i + 2) >> 1) & 0x7F) << 15) | ((uint64_t)DATA(i + 3) << 7) | ((DATA(i + 4) >> 1) & 0x7F);
        i += 5;
    }
    if (0x01 & pes->PTS_DTS_flags) {
        // DTS
        assert(0x10 == (DATA(i) & 0x10));
        pes->dts = ((((uint64_t)DATA(i) >> 1) & 0x07) << 30) | ((uint64_t)DATA(i + 1) << 22) | ((((uint64_t)DATA(i + 2) >> 1) & 0x7F) << 15) | ((uint64_t)DATA(i + 3) << 7) | ((DATA(i + 4) >> 1) & 0x7F);
        i += 5;
    } else if (0x02 & pes->PTS_DTS_flags) {
        pes->dts = pes->pts;  // 如果只有 0x10 的话  那说明 没有dts的位置  那就是 DTS=PTS
    }

    if (pes->DSM_trick_mode_flag) i+=1;
    if (pes->additional_copy_info_flag) i+=1;
    if (pes->PES_CRC_flag) i += 2;
    if (pes->PES_extension_flag) {}
    if (pes->len) {
        if (pes->len < pes->PES_header_data_length + 3) {
            return 0;
        }
        pes->len -= pes->PES_header_data_length + 3;
    }
    assert(pes->len >= 0);

	return overflow ? 0 : pes->PES_header_data_length + 9;
}


void read_file(const char* path) {
	FILE *fp = fopen(path, "r");
	if (fp == NULL) {
		printf("open %s failed, errno=%d\n", path, errno);
		return;
	}
	ts_demuxer muxer;
	memset(&muxer, 0, sizeof(muxer));
	memset(&muxer.pat, 0, sizeof(muxer.pat));
	char buf[188] = { 0 };
	int count = 0;
    while (true)
	{
		size_t n = fread(buf, 1, sizeof(buf), fp);
		if (feof(fp)) break;
		assert(n == 188);
		ts_demuxer_input(&muxer, (uint8_t*)buf, n);
        count++;
	}
	fclose(fp);
    printf("packet count = %d\n", count);
}


int main(int argc, char** argv) {
	if (argc != 2) {
		fprintf(stderr, "invalid command argments\n");
		return -1;
	}
	read_file(argv[1]);

	printf("parse finished\n");
	return 0;
}
