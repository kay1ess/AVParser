

/*
解析MPEG-TS

1.从复用的MPEG-TS流中解析出TS包；
2.从TS包中获取PAT及对应的PMT（PSI中的表格）；
3.从而获取特定节目的音视频PID；
4.通过PID筛选出特定音视频相关的TS包，并解析出PES；
5.从PES中读取到PTS/DTS，并从PES中解析出基本码流ES；
6.将ES交给解码器，获得压缩前的原始音视频数据

*/


#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define TS_PACKET_SIZE 188
#define TS_SYNC_BYTE 0x47

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
	unsigned stream_type : 8;
	unsigned elementary_PID : 13;
	unsigned ES_info_length : 12;
	unsigned descriptor;
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

size_t pat_read(ts_pat* pat, const uint8_t* data, size_t bytes);
size_t sdt_read(ts_pat* pat, const uint8_t* data, size_t bytes);
size_t pmt_read(ts_pmt* pmt, const uint8_t* data, size_t bytes);

int ts_demuxer_input(ts_demuxer *ts, const uint8_t* data, size_t bytes) {
	assert(188 == bytes);
	assert(0x47 == data[0]);

	ts_header pkhd;
	uint32_t i;

	uint32_t PID;

	PID = (data[1] << 8 | data[2]) & 0x1FFF;
	memset(&pkhd, 0, sizeof(pkhd));
	pkhd.transport_error_indicator = (data[1] >> 7) & 0x01;
	pkhd.payload_unit_start_indicator = (data[1] >> 6) & 0x01;
	pkhd.transport_priority = (data[1] >> 5) & 0x01;
	pkhd.transport_scrambling_control = (data[3] >> 6) & 0x03;
	pkhd.adaptation_field_control = (data[3] >> 4) & 0x03;
	pkhd.continuity_counter = (data[3]) & 0x0F;

	if ((ts->cc + 1) % 15 != (uint8_t)pkhd.continuity_counter) {
		// 如果包不是连续的 
	}

	ts->cc = (uint8_t)pkhd.continuity_counter;

	// 打印出每个包的头部信息

	i = 4;
	if (0x02 == pkhd.adaptation_field_control) {
		// 仅有适应域
	}

	if (0x01 == pkhd.adaptation_field_control) {
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
		case TS_PID_CAT:
			// CAT
		case TS_PID_TSDT:
			// TSDT
		case TS_PID_IPMP:
			// IPMP
		case TS_PID_NIT:
			// NIT
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
			if (pat->pat_programs[p].program_number = service_id) {
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
		desc = 0x00;
		if (len != 0) {
			// descriptor
			// 这里忽略
		}
		ts_pes st;
		st.stream_type = stream_type;
		st.elementary_PID = pid;
		st.ES_info_length = len;
		st.descriptor = desc;
		pmt->pmt_streams.push_back(st);
	}

	return 0;
}


size_t pes_read_header(ts_pes* pes, const uint8_t* data, size_t bytes) {
	return 0;
}


void read_file(const char* path) {
	FILE *fp = fopen(path, "r");
	if (fp == NULL) {
		printf("open %s failed, errno=%d\n", fp, errno);
		return;
	}
	ts_demuxer muxer;
	memset(&muxer, 0, sizeof(muxer));
	char buf[188] = { 0 };
	while (true)
	{
		size_t n = fread(buf, 1, sizeof(buf), fp);
		if (feof(fp)) break;
		assert(n == 188);
		ts_demuxer_input(&muxer, (uint8_t*)buf, n);
	}
	fclose(fp);
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