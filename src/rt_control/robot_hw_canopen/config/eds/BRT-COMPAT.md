# BRT 多圈编码器兼容文件

厂家原件 `BRT-Multi-turn-V1.0.eds` 保持原始SHA256；兼容文件不是厂家新版本。

- `6502:00` 改为UNSIGNED32。四个实物节点均返回4字节，BRT协议v2.6第14页也定义为Unsigned32。
- 为适配Lely的CiA301严格分类，将五个DS406对象从MandatoryObjects移到OptionalObjects。对象仍全部保留，不改变设备行规对它们的要求。
- 对象默认值、访问权限、PDO映射及其他类型不变。分辨率65536、16圈范围和标定偏置由实机配置单独记录，不将当前读数伪装成厂家默认值。

差异见同目录compat.diff，来源和哈希见compat.yaml。生成DCF仍使用严格检查，不使用no-strict。
