const Sequelize = require('sequelize');

module.exports = class User extends Sequelize.Model {
  // 테이블에 대한 설정
  static init(sequelize) {
    return super.init({ // 테이블 컬럼 설정
      // id를 자동으로 기본 키로 연결
      name: {
        type: Sequelize.STRING(20),
        allowNull: false,
        unique: true,
      },
      age: {
        type: Sequelize.INTEGER.UNSIGNED,
        allowNull: false,
      },
      married: {
        type: Sequelize.BOOLEAN,
        allowNull: false,
      },
      comment: {
        type: Sequelize.TEXT,
        allowNull: true,
      },
      created_at: {
        type: Sequelize.DATE,
        allowNull: false,
        defaultValue: Sequelize.NOW,
      },
    }, {  // 테이블 자체 설정
      sequelize,
      timestamps: false,
      underscored: false,
      modelName: 'User',
      tableName: 'users',
      paranoid: false,    // true => deletedAt 컬럼 생성 => 복원을 위한 옵션
      charset: 'utf8',
      collate: 'utf8_general_ci',
    });
  }
  // 다른 모델과의 관계
  static associate(db) {
    db.User.hasMany(db.Comment, { foreignKey: 'commenter', sourceKey: 'id' });
  }
};