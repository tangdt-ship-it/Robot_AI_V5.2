#ifndef MCP_SERVER_H
#define MCP_SERVER_H

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <variant>
#include <optional>
#include <stdexcept>
#include <thread>
#include <mbedtls/base64.h>

#include <cJSON.h>

class ImageContent {
private:
    std::string encoded_data_;
    std::string mime_type_;

    static std::string Base64Encode(const std::string& data) {
        size_t dlen = 0, olen = 0;
        mbedtls_base64_encode((unsigned char*)nullptr, 0, &dlen, (const unsigned char*)data.data(), data.size());
        std::string result(dlen, 0);
        mbedtls_base64_encode((unsigned char*)result.data(), result.size(), &olen, (const unsigned char*)data.data(), data.size());
        return result;
    }

public:
    ImageContent(const std::string& mime_type, const std::string& data) {
        mime_type_ = mime_type;
        // base64 encode data
        encoded_data_ = Base64Encode(data);
    }

    std::string to_json() const {
        cJSON *json = cJSON_CreateObject();
        cJSON_AddStringToObject(json, "type", "image");
        cJSON_AddStringToObject(json, "mimeType", mime_type_.c_str());
        cJSON_AddStringToObject(json, "data", encoded_data_.c_str());
        char* json_str = cJSON_PrintUnformatted(json);
        std::string result(json_str);
        cJSON_free(json_str);
        cJSON_Delete(json);
        return result;
    }
};

// 添加类型别名
using ReturnValue = std::variant<bool, int, std::string, cJSON*, ImageContent*>;

enum PropertyType {
    kPropertyTypeBoolean,
    kPropertyTypeInteger,
    kPropertyTypeString
};

class Property {
private:
    std::string name_;
    PropertyType type_;
    std::variant<bool, int, std::string> value_;
    bool has_default_value_;
    std::optional<int> min_value_;  // 新增：整数最小值
    std::optional<int> max_value_;  // 新增：整数最大值

public:
    // Required field constructor
    Property(const std::string& name, PropertyType type)
        : name_(name), type_(type), has_default_value_(false) {}

    // Optional field constructor with default value
    template<typename T>
    Property(const std::string& name, PropertyType type, const T& default_value)
        : name_(name), type_(type), has_default_value_(true) {
        value_ = default_value;
    }

    Property(const std::string& name, PropertyType type, int min_value, int max_value)
        : name_(name), type_(type), has_default_value_(false), min_value_(min_value), max_value_(max_value) {
        if (type != kPropertyTypeInteger) {
            throw std::invalid_argument("Range limits only apply to integer properties");
        }
    }

    Property(const std::string& name, PropertyType type, int default_value, int min_value, int max_value)
        : name_(name), type_(type), has_default_value_(true), min_value_(min_value), max_value_(max_value) {
        if (type != kPropertyTypeInteger) {
            throw std::invalid_argument("Range limits only apply to integer properties");
        }
        if (default_value < min_value || default_value > max_value) {
            throw std::invalid_argument("Default value must be within the specified range");
        }
        value_ = default_value;
    }

    inline const std::string& name() const { return name_; }
    inline PropertyType type() const { return type_; }
    inline bool has_default_value() const { return has_default_value_; }
    inline bool has_range() const { return min_value_.has_value() && max_value_.has_value(); }
    inline int min_value() const { return min_value_.value_or(0); }
    inline int max_value() const { return max_value_.value_or(0); }

    template<typename T>
    inline T value() const {
        return std::get<T>(value_);
    }

    template<typename T>
    inline void set_value(const T& value) {
        // 添加对设置的整数值进行范围检查
        if constexpr (std::is_same_v<T, int>) {
            if (min_value_.has_value() && value < min_value_.value()) {
                throw std::invalid_argument("Value is below minimum allowed: " + std::to_string(min_value_.value()));
            }
            if (max_value_.has_value() && value > max_value_.value()) {
                throw std::invalid_argument("Value exceeds maximum allowed: " + std::to_string(max_value_.value()));
            }
        }
        value_ = value;
    }

    std::string to_json() const {
        cJSON *json = cJSON_CreateObject();
        
        if (type_ == kPropertyTypeBoolean) {
            cJSON_AddStringToObject(json, "type", "boolean");
            if (has_default_value_) {
                cJSON_AddBoolToObject(json, "default", value<bool>());
            }
        } else if (type_ == kPropertyTypeInteger) {
            cJSON_AddStringToObject(json, "type", "integer");
            if (has_default_value_) {
                cJSON_AddNumberToObject(json, "default", value<int>());
            }
            if (min_value_.has_value()) {
                cJSON_AddNumberToObject(json, "minimum", min_value_.value());
            }
            if (max_value_.has_value()) {
                cJSON_AddNumberToObject(json, "maximum", max_value_.value());
            }
        } else if (type_ == kPropertyTypeString) {
            cJSON_AddStringToObject(json, "type", "string");
            if (has_default_value_) {
                cJSON_AddStringToObject(json, "default", value<std::string>().c_str());
            }
        }
        
        char *json_str = cJSON_PrintUnformatted(json);
        std::string result(json_str);
        cJSON_free(json_str);
        cJSON_Delete(json);
        
        return result;
    }
};

class PropertyList {
private:
    std::vector<Property> properties_;
    uint32_t request_generation_ = 0U;

public:
    PropertyList() = default;
    PropertyList(const std::vector<Property>& properties) : properties_(properties) {}
    void AddProperty(const Property& property) {
        properties_.push_back(property);
    }
    void SetRequestGeneration(uint32_t generation) {
        request_generation_ = generation;
    }
    uint32_t RequestGeneration() const { return request_generation_; }

    const Property& operator[](const std::string& name) const {
        for (const auto& property : properties_) {
            if (property.name() == name) {
                return property;
            }
        }
        throw std::runtime_error("Property not found: " + name);
    }

    auto begin() { return properties_.begin(); }
    auto end() { return properties_.end(); }

    std::vector<std::string> GetRequired() const {
        std::vector<std::string> required;
        for (auto& property : properties_) {
            if (!property.has_default_value()) {
                required.push_back(property.name());
            }
        }
        return required;
    }

    std::string to_json() const {
        cJSON *json = cJSON_CreateObject();
        
        for (const auto& property : properties_) {
            cJSON *prop_json = cJSON_Parse(property.to_json().c_str());
            cJSON_AddItemToObject(json, property.name().c_str(), prop_json);
        }
        
        char *json_str = cJSON_PrintUnformatted(json);
        std::string result(json_str);
        cJSON_free(json_str);
        cJSON_Delete(json);
        
        return result;
    }
};

class McpTool {
private:
    std::string name_;
    std::string description_;
    PropertyList properties_;
    std::function<ReturnValue(const PropertyList&)> callback_;
    bool user_only_ = false;

    // Robot AI V5.2.2 MCP cleanup policy.
    // Keep legacy registrations in source for rollback/history, but remove
    // overlapping or misleading motion aliases from the normal AI surface and
    // make direct legacy calls fail closed instead of issuing motor commands.
    // Motor, RobotLink, HOME, MAP and STM32 safety implementations are not
    // changed by this policy.
    static bool HideLegacyRobotMotionToolFromAi(const std::string& name) {
        return name == "self.robot.turn_and_move" ||
               name == "self.robot.navigate_autonomously" ||
               name == "self.robot.cancel_mission" ||
               name == "self.robot.move_forward" ||
               name == "self.robot.move_backward" ||
               name == "self.robot.turn_left" ||
               name == "self.robot.turn_right" ||
               name == "self.robot.rotate_continuous";
    }

    static const char* RobotMotionDescriptionOverride(const std::string& name) {
        if (name == "self.robot.stop") {
            return "Dừng robot ngay lập tức với ưu tiên cao nhất. Dùng cho các cách nói như dừng, dừng lại, đứng lại, ngừng, ngừng lại, stop. STOP phải hủy mission/chuyển động đang chạy và chỉ báo hoàn tất sau khi STM32 xác nhận motor đã dừng.";
        }
        if (name == "self.robot.move_distance") {
            return "Tool chuẩn duy nhất cho tiến/lùi có quãng đường. Chuyển đổi mọi đơn vị sang distance_mm trước khi gọi: 1 cm=10 mm, 1 m=1000 mm, và quy ước 1 bước=5 cm=50 mm. forward=true cho các cách nói tiến, đi thẳng, tiến về phía trước, đi lên, tiến lên, tiến thẳng, di chuyển lên/trước; forward=false cho lùi, lùi về sau, thụt lại, đi lùi, di chuyển về sau. Robot dùng encoder và chỉ được xác nhận đi đủ khi completed=true.";
        }
        if (name == "self.robot.turn_relative") {
            return "Tool chuẩn duy nhất cho quay/xoay/rẽ/quẹo theo góc tương đối. direction=left/right, degrees=1..180. Các từ xoay, quay, rẽ, quẹo, nghiêng đều ánh xạ vào tool này khi có số độ. 'Quay đầu' nghĩa là 180 độ; nếu người dùng nói rõ trái/phải thì dùng hướng đó. Không dùng turn_left/turn_right/rotate_continuous vì các alias đó đã bị khóa khỏi AI.";
        }
        if (name == "self.robot.turn_revolutions") {
            return "Tool quay theo so vong tai cho: direction=left/right, revolutions=1..2, 1 vong=360 do. Cac cach noi quay trai/phai 1 vong, xoay 2 vong deu dung tool nay. Moi vong duoc thuc hien bang hai doan quay kin 180 do bang Heading; chi bao hoan tat khi completed=true. Neu nguoi dung noi quay trai 2 vong roi quay phai 2 vong, goi tool nay hai lan theo dung thu tu va chi goi lan sau sau khi lan truoc completed=true. STOP co the huy giua cac doan; khong dung turn_left/turn_right/rotate_continuous.";
        }
        if (name == "self.robot.turn_to_heading") {
            return "Quay robot tới heading tuyệt đối -180..180 độ bằng fused heading. Chỉ dùng khi người dùng yêu cầu một hướng tuyệt đối, ví dụ quay về hướng 0 độ; không dùng thay cho quay trái/phải một số độ tương đối.";
        }
        if (name == "self.robot.set_home") {
            return "Đặt vị trí hiện tại làm HOME/điểm xuất phát. Dùng cho các cách nói set home, đặt vị trí nhà, đặt nhà, đặt vị trí xuất phát, đánh dấu điểm xuất phát. Tool không làm robot di chuyển và chỉ thành công khi odometry sẵn sàng, robot không có mission đang chạy.";
        }
        if (name == "self.robot.return_home") {
            return "Cho robot trở về HOME đã lưu. Dùng cho các cách nói về nhà, quay về nhà, quay về chỗ cũ, trở lại vị trí ban đầu, quay về điểm xuất phát. Đây là mission Return Home riêng; chỉ xác nhận đã về khi mission state là return_completed.";
        }
        if (name == "self.robot.scan_obstacle") {
            return "PHYSICAL MOTION: robot sẽ thật sự xoay thân trái/phải để quét môi trường bằng HC-SR04 + camera rồi trở lại hướng gốc. Chỉ dùng khi người dùng yêu cầu quét/nhìn xung quanh hoặc kiểm tra hai bên. Nếu chỉ hỏi vật cản phía trước, dùng get_diagnostics(target=obstacle) vì tool đó không làm robot di chuyển.";
        }
        return nullptr;
    }

    void ApplyRobotAiMotionPolicy(const std::string& original_name) {
        if (HideLegacyRobotMotionToolFromAi(original_name)) {
            user_only_ = true;
            callback_ = [original_name](const PropertyList&) -> ReturnValue {
                return std::string(
                    "{\"completed\":false,\"accepted\":false,\"error\":\"legacy_motion_tool_disabled\",\"tool\":\"" +
                    original_name +
                    "\",\"use\":\"stop/move_distance/turn_relative/turn_to_heading/set_home/return_home\"}");
            };
        }
        const char* override_text = RobotMotionDescriptionOverride(original_name);
        if (override_text != nullptr) {
            description_ = override_text;
        }
    }

public:
    McpTool(const std::string& name, 
            const std::string& description, 
            const PropertyList& properties, 
            std::function<ReturnValue(const PropertyList&)> callback)
        : name_(name), 
        description_(description), 
        properties_(properties), 
        callback_(callback) {
        ApplyRobotAiMotionPolicy(name);
    }

    void set_user_only(bool user_only) { user_only_ = user_only; }
    inline const std::string& name() const { return name_; }
    inline const std::string& description() const { return description_; }
    inline const PropertyList& properties() const { return properties_; }
    inline bool user_only() const { return user_only_; }

    std::string to_json() const {
        std::vector<std::string> required = properties_.GetRequired();
        
        cJSON *json = cJSON_CreateObject();
        cJSON_AddStringToObject(json, "name", name_.c_str());
        cJSON_AddStringToObject(json, "description", description_.c_str());
        
        cJSON *input_schema = cJSON_CreateObject();
        cJSON_AddStringToObject(input_schema, "type", "object");
        
        cJSON *properties = cJSON_Parse(properties_.to_json().c_str());
        cJSON_AddItemToObject(input_schema, "properties", properties);
        
        if (!required.empty()) {
            cJSON *required_array = cJSON_CreateArray();
            for (const auto& property : required) {
                cJSON_AddItemToArray(required_array, cJSON_CreateString(property.c_str()));
            }
            cJSON_AddItemToObject(input_schema, "required", required_array);
        }
        
        cJSON_AddItemToObject(json, "inputSchema", input_schema);

        // Add audience annotation if the tool is user only (invisible to AI)
        if (user_only_) {
            cJSON *annotations = cJSON_CreateObject();
            cJSON *audience = cJSON_CreateArray();
            cJSON_AddItemToArray(audience, cJSON_CreateString("user"));
            cJSON_AddItemToObject(annotations, "audience", audience);
            cJSON_AddItemToObject(json, "annotations", annotations);
        }
        
        char *json_str = cJSON_PrintUnformatted(json);
        std::string result(json_str);
        cJSON_free(json_str);
        cJSON_Delete(json);
        
        return result;
    }

    std::string Call(const PropertyList& properties) {
        ReturnValue return_value = callback_(properties);
        // 返回结果
        cJSON* result = cJSON_CreateObject();
        cJSON* content = cJSON_CreateArray();

        if (std::holds_alternative<ImageContent*>(return_value)) {
            auto image_content = std::get<ImageContent*>(return_value);
            cJSON* image = cJSON_CreateObject();
            cJSON_AddStringToObject(image, "type", "image");
            cJSON_AddStringToObject(image, "image", image_content->to_json().c_str());
            cJSON_AddItemToArray(content, image);
            delete image_content;
        } else {
            cJSON* text = cJSON_CreateObject();
            cJSON_AddStringToObject(text, "type", "text");
            if (std::holds_alternative<std::string>(return_value)) {
                cJSON_AddStringToObject(text, "text", std::get<std::string>(return_value).c_str());
            } else if (std::holds_alternative<bool>(return_value)) {
                cJSON_AddStringToObject(text, "text", std::get<bool>(return_value) ? "true" : "false");
            } else if (std::holds_alternative<int>(return_value)) {
                cJSON_AddStringToObject(text, "text", std::to_string(std::get<int>(return_value)).c_str());
            } else if (std::holds_alternative<cJSON*>(return_value)) {
                cJSON* json = std::get<cJSON*>(return_value);
                char* json_str = cJSON_PrintUnformatted(json);
                cJSON_AddStringToObject(text, "text", json_str);
                cJSON_free(json_str);
                cJSON_Delete(json);
            }
            cJSON_AddItemToArray(content, text);
        }
        cJSON_AddItemToObject(result, "content", content);
        cJSON_AddBoolToObject(result, "isError", false);

        auto json_str = cJSON_PrintUnformatted(result);
        std::string result_str(json_str);
        cJSON_free(json_str);
        cJSON_Delete(result);
        return result_str;
    }
};

class McpServer {
public:
    using RequestGenerationProvider =
        std::function<uint32_t(const std::string&)>;

    static McpServer& GetInstance() {
        static McpServer instance;
        return instance;
    }

    void AddCommonTools();
    void AddUserOnlyTools();
    void AddTool(McpTool* tool);
    void AddTool(const std::string& name, const std::string& description, const PropertyList& properties, std::function<ReturnValue(const PropertyList&)> callback);
    void AddUserOnlyTool(const std::string& name, const std::string& description, const PropertyList& properties, std::function<ReturnValue(const PropertyList&)> callback);
    void SetRequestGenerationProvider(RequestGenerationProvider provider) {
        request_generation_provider_ = std::move(provider);
    }
    void ParseMessage(const cJSON* json);
    void ParseMessage(const std::string& message);

private:
    McpServer();
    ~McpServer();

    void ParseCapabilities(const cJSON* capabilities);

    void ReplyResult(int id, const std::string& result);
    void ReplyError(int id, const std::string& message);

    void GetToolsList(int id, const std::string& cursor, bool list_user_only_tools);
    void DoToolCall(int id, const std::string& tool_name, const cJSON* tool_arguments);

    std::vector<McpTool*> tools_;
    RequestGenerationProvider request_generation_provider_;
};

#endif // MCP_SERVER_H
